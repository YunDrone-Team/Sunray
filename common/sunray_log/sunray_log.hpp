#ifndef SUNRAY_LOG_HPP
#define SUNRAY_LOG_HPP

// -----------------------------------------------------------------------------
// 使用方式:
// 1.在工程 CMakeLists.txt 中，将以下路径加入 include_directories：
//     Sunray/common/sunray_log/
//     Sunray/common/sunray_log/spdlog/include
// 2.在需要使用日志的文件中包含头文件：
//     #include "sunray_log.hpp"
// 具体使用参考 sunray_log_example 功能包和 spdlog 库官方文档
// -----------------------------------------------------------------------------

#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

// 必须在包含 spdlog 头文件之前定义，否则 SPDLOG_LOGGER_DEBUG/TRACE 宏会在编译期被剔除
#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

// =============================================================================
// 日志级别枚举
// =============================================================================
enum class SunrayLogLevel {
    trace = spdlog::level::trace,
    debug = spdlog::level::debug,
    info = spdlog::level::info,
    warn = spdlog::level::warn,
    error = spdlog::level::err,
    critical = spdlog::level::critical
};

// =============================================================================
// 日志器配置结构体
// =============================================================================
struct SunrayLogConfig {
    // logger 名称，显示在每条日志中
    std::string name = "sunray";

    // 控制台输出级别
    SunrayLogLevel console_level = SunrayLogLevel::info;

    // 日志文件路径，为空字符串时不输出到文件
    std::string file_path = "";

    // 文件输出级别（通常比控制台更详细）
    SunrayLogLevel file_level = SunrayLogLevel::trace;

    // 是否启用异步模式
    bool async = false;

    // 异步队列大小（async=true 时生效）
    std::size_t async_queue_size = 8192;

    // 异步后台线程数（async=true 时生效）
    std::size_t async_thread_count = 1;
};

// =============================================================================
// SunrayLogger — 全局单例日志器
// =============================================================================
class SunrayLogger {
  public:
    static SunrayLogger& instance() {
        static SunrayLogger inst;
        return inst;
    }

    // 根据配置初始化（可在 main() 中调用；不调用则使用默认控制台输出）
    void Init(const SunrayLogConfig& cfg) {

        std::vector<spdlog::sink_ptr> sinks;

        auto to_spdlog = [](SunrayLogLevel lvl) { return static_cast<spdlog::level::level_enum>(lvl); };

        // 控制台 sink（彩色）
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(to_spdlog(cfg.console_level));

        // console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] [%s:%#] %v");
        console_sink->set_pattern("[%^%l%$] %v");
        sinks.push_back(console_sink);

        // 文件 sink（可选，每次启动覆盖写入）
        if (!cfg.file_path.empty()) {
            // 若目录不存在则自动创建
            std::string dir = GetParentPath(cfg.file_path);
            if (!dir.empty()) {
                CreateDirectories(dir);
            }
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(cfg.file_path, /*truncate=*/true);
            file_sink->set_level(to_spdlog(cfg.file_level));
            // file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] [%s:%#] %v");
            file_sink->set_pattern("[%l] %v");
            sinks.push_back(file_sink);
        }

        // logger 整体 level 取两者最低，保证低级别消息能到达对应 sink
        spdlog::level::level_enum min_level = cfg.file_path.empty()
                                                  ? to_spdlog(cfg.console_level)
                                                  : std::min(to_spdlog(cfg.console_level), to_spdlog(cfg.file_level));

        if (cfg.async) {
            spdlog::init_thread_pool(cfg.async_queue_size, cfg.async_thread_count);
            logger_ = std::make_shared<spdlog::async_logger>(
                cfg.name, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
        } else {
            logger_ = std::make_shared<spdlog::logger>(cfg.name, sinks.begin(), sinks.end());
        }

        logger_->set_level(min_level);
        // 每条消息立即刷新（可按需改为 flush_on 或 flush_every）
        logger_->flush_on(spdlog::level::trace);

        // 若同名 logger 已注册则先删除再重新注册
        spdlog::drop(cfg.name);
        spdlog::register_logger(logger_);
    }

    std::shared_ptr<spdlog::logger> Get() const {
        return logger_;
    }

  private:
    SunrayLogger() {
        // 默认初始化：不调用 Init() 直接使用宏也不会崩溃
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        // console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] [%s:%#] %v");
        console_sink->set_pattern("[%^%l%$] %v");
        logger_ = std::make_shared<spdlog::logger>("sunray", console_sink);
        logger_->set_level(spdlog::level::info);
        logger_->flush_on(spdlog::level::trace);
    }

    std::shared_ptr<spdlog::logger> logger_;

    // 递归创建目录（兼容 C++11）
    void CreateDirectories(const std::string& path) {
        std::string tmp = path;
        for (std::size_t i = 1; i < tmp.size(); ++i) {
            if (tmp[i] == '/') {
                tmp[i] = '\0';
                ::mkdir(tmp.c_str(), 0755);
                tmp[i] = '/';
            }
        }
        ::mkdir(tmp.c_str(), 0755);
    }

    std::string GetParentPath(const std::string& file_path) {
        auto pos = file_path.rfind('/');
        return (pos == std::string::npos) ? "" : file_path.substr(0, pos);
    }
};

// =============================================================================
// 宏定义（基于 SPDLOG_LOGGER_* 系列，自动携带文件名和行号）
// =============================================================================
#define SUNRAY_TRACE(...) SPDLOG_LOGGER_TRACE(SunrayLogger::instance().Get(), __VA_ARGS__)
#define SUNRAY_DEBUG(...) SPDLOG_LOGGER_DEBUG(SunrayLogger::instance().Get(), __VA_ARGS__)
#define SUNRAY_INFO(...) SPDLOG_LOGGER_INFO(SunrayLogger::instance().Get(), __VA_ARGS__)
#define SUNRAY_WARN(...) SPDLOG_LOGGER_WARN(SunrayLogger::instance().Get(), __VA_ARGS__)
#define SUNRAY_ERROR(...) SPDLOG_LOGGER_ERROR(SunrayLogger::instance().Get(), __VA_ARGS__)
#define SUNRAY_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(SunrayLogger::instance().Get(), __VA_ARGS__)

#endif  // SUNRAY_LOG_HPP