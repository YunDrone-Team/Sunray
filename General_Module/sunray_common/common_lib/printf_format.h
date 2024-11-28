#ifndef PRINTF_FORMAT_H
#define PRINTF_FORMAT_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <chrono>
#include <mutex>
#include <iomanip>
#include <condition_variable>
#include <thread>
#include <queue>
#include <chrono>
#include <ctime>
#include <functional>

#define LOG_DEFAULT "\033[0m"              // 默认颜色
#define LOG_RED "\033[31m"                 // 红色
#define LOG_GREEN "\033[32m"               // 绿色
#define LOG_YELLOW "\033[33m"              // 黄色
#define LOG_BLUE "\033[34m"                // 蓝色
#define LOG_MAGENTA "\033[35m"             // 紫色
#define LOG_CYAN "\033[36m"                // 青色
#define LOG_WHITE "\033[37m"               // 白色
#define LOG_BLACK "\033[30m"               // 黑色
#define LOG_BOLD "\033[1m"                 // 加粗
#define LOG_UNDERLINE "\033[4m"            // 下划线
#define LOG_BLINK "\033[5m"                // 闪烁
#define LOG_INVERT "\033[7m"               // 反显
#define LOG_HIDDEN "\033[8m"               // 隐藏
#define LOG_CLEAR "\033[2J"                // 清屏
#define LOG_WHITE_BG_GREEN "\033[47;32m"   // 白底绿字
#define LOG_WHITE_BG_RED "\033[47;31m"     // 白底红字
#define LOG_WHITE_BG_YELLOW "\033[47;33m"  // 白底黄字
#define LOG_WHITE_BG_BLUE "\033[47;34m"    // 白底蓝字
#define LOG_WHITE_BG_MAGENTA "\033[47;35m" // 白底紫字
#define LOG_WHITE_BG_CYAN "\033[47;36m"    // 白底青字
#define LOG_WHITE_BG_BLACK "\033[47;30m"   // 白底黑字
#define LOG_BLACK_BG_WHITE "\033[40;37m"   // 黑底白字
#define LOG_BLACK_BG_RED "\033[40;31m"     // 黑底红字
#define LOG_BLACK_BG_YELLOW "\033[40;33m"  // 黑底黄字
#define LOG_BLACK_BG_BLUE "\033[40;34m"    // 黑底蓝字
#define LOG_BLACK_BG_MAGENTA "\033[40;35m" // 黑底紫字
#define LOG_BLACK_BG_CYAN "\033[40;36m"    // 黑底青字
#define LOG_RESET "\033[0m"                // 重置颜色

// 日志级别
enum class LogLevel
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    QUIET
};

// 定义颜色枚举
enum class LogColor
{
    def = 0,
    red,
    green,
    yellow,
    blue,
    magenta,
    cyan,
    white,
    black,
    bold,
    underline,
    blink,
    invert,
    hidden,
    clear,
    white_bg_green,
    white_bg_red,
    white_bg_yellow,
    white_bg_blue,
    white_bg_magenta,
    white_bg_cyan,
    white_bg_black,
    black_bg_white,
    black_bg_red,
    black_bg_yellow,
    black_bg_blue,
    black_bg_magenta,
    black_bg_cyan
};

// 日志类命名空间，实现静态类
namespace sunray_logger
{
    // 日志类
    class Logger
    {
    public:
        // 构造函数
        Logger() = delete;
        // 析构函数
        ~Logger() = delete;

        // 创建颜色数组
        static std::array<std::string, 28> colors;
        // 创建日志级别数组
        static std::array<std::string, 5> levels;

        static int precision;             // 小数点精度
        static bool printLevel;           // 是否打印level
        static bool printTime;            // 是否打印时间
        static bool printToFile;          // 是否输出到文件
        static bool is_openFile;          // 是否打开了文件
        static bool is_init;              // 是否初始化
        static std::string delimiter;     // 分隔符
        static std::string printColor;    // 设置默认颜色
        static std::string printLevelStr; // 设置默认level
        static std::string fileName;      // 文件路径+文件名
        static std::fstream f_stream;     // 文件输出流对象

        // 初始化分隔符
        static void setSeparator(const char *se)
        {
            delimiter = se;
        }
        // 初始化小数点精度
        static void setPrecision(int p)
        {
            precision = p;
        }
        // 是否打印level
        static void setPrintLevel(bool p)
        {
            printLevel = p;
        }
        // 是否打印时间
        static void setPrintTime(bool p)
        {
            printTime = p;
        }
        // 是否输出到文件
        static void setPrintToFile(bool p)
        {
            printToFile = p;
        }
        // 初始化默认颜色
        static void setDefaultColor(const char *c)
        {
            printColor = c;
        }
        // 初始化默认level
        static void setDefaultLevel(int l)
        {
            if (l >= 1 && l <= static_cast<int>(levels.size()))
            {
                printLevelStr = levels[l - 1];
            }
            else
            {
                printLevelStr = levels[0];
            }
        }
        // 初始化默认颜色
        static void setDefaultColor(int c)
        {
            if (c >= 1 && c <= static_cast<int>(colors.size()))
            {
                printColor = colors[c - 1];
            }
            else
            {
                printColor = LOG_DEFAULT;
            }
        }
        // 初始化文件名
        static void setFilename(const char *f)
        {
            fileName = f;
        }

        // 浮点数格式化
        static const char *float_format(std::ostringstream &oss, float value)
        {
            // 截断小数点
            oss << std::fixed << std::setprecision(precision) << value;
        }
        // double格式化
        static const char *double_format(std::ostringstream &oss, double value)
        {
            // 截断小数点
            oss << std::fixed << std::setprecision(precision) << value;
        }

        // 获取当前时间
        static std::string get_current_timestamp()
        {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            std::stringstream ss;
            ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << ms.count();
            return ss.str();
        }

        // 辅助函数：用于递归地拼接参数
        template <typename T>
        static void append_to_stream(std::ostringstream &oss, T value)
        {
            

            // if constexpr (std::is_same<T, double>::value)
            // {
            //     float_format(oss, value);
            // }
            // else if constexpr (std::is_same<T, double>::value)
            // {
            //     double_format(oss, value);
            // }
            // else
            // {
            //     oss << value;
            // }
            oss << value;
        }

        template <typename T, typename... Args>
        static void append_to_stream(std::ostringstream &oss, T first, Args... args)
        {
            oss << first << delimiter;
            append_to_stream(oss, args...);
        }

        // 主函数：接受1到N个参数，并输出拼接后的字符串
        template <typename... Args>
        static void print(Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << printColor;
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << printLevelStr << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;

            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(4) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(4) << std::endl;
                    }
                }
            }
        }

        // 主函数：接受颜色 + 1到N个参数，并输出拼接后的字符串
        template <typename... Args>
        static void print_color(int color, Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << colors[color];
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << printLevelStr << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;

            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[color].size()) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[color].size()) << std::endl;
                    }
                }
            }
        }

        // 主函数：接受颜色 + 1到N个参数，并输出拼接后的字符串 不拼接分隔符
        template <typename... Args>
        static void color_no_del(int color, Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << colors[color];
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << printLevelStr << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            std::string tmp_delimiter = delimiter;
            setSeparator("");
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;
            setSeparator(tmp_delimiter.c_str());
            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[color].size()) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[color].size()) << std::endl;
                    }
                }
            }
        }


        // INFO
        template <typename... Args>
        static void info(Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << colors[int(LogColor::green)];
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << levels[int(LogLevel::INFO)] << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;

            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::green)].size()) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::green)].size()) << std::endl;
                    }
                }
            }
        }

        // ERROR
        template <typename... Args>
        static void error(Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << colors[int(LogColor::red)];
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << levels[int(LogLevel::ERROR)] << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;

            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::red)].size()) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::red)].size()) << std::endl;
                    }
                }
            }
        }

        // WARNING
        template <typename... Args>
        static void warning(Args... args)
        {
            if (!is_init)
            {
                throw std::logic_error("printf_format not init");
                return;
            }
            std::ostringstream oss;
            // 强制显示正负号
            oss << std::showpos;
            // 设置小数点精度
            oss << std::fixed << std::setprecision(precision);
            // 添加颜色
            oss << colors[int(LogColor::yellow)];
            // 判断level和时间是否需要打印
            if (printLevel)
            {
                oss << "[" << levels[int(LogLevel::WARN)] << "]";
            }
            if (printTime)
            {
                oss << get_current_timestamp() << ":";
            }
            // 拼接分隔符
            oss << delimiter;
            append_to_stream(oss, args...);
            std::cout << oss.str() << std::endl;

            // 判断是否要输出到文件
            if (printToFile)
            {
                // 判断文件输入已经打开
                if (is_openFile)
                {
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::yellow)].size()) << std::endl;
                    }
                }
                else
                {
                    createLogFile();
                    if (!f_stream.fail())
                    {
                        f_stream << oss.str().substr(colors[int(LogColor::yellow)].size()) << std::endl;
                    }
                }
            }
        }

        // 主函数：带有分隔符和颜色，接受1到N个参数，并输出拼接后的字符串
        // template <typename... Args>
        // static void print(std::string se, int color, Args... args)
        // {
        //     if (!is_init)
        //     {
        //         throw std::logic_error("printf_format not init");
        //         return;
        //     }
        //     std::ostringstream oss;
        //     append_to_stream(oss, args...);
        //     std::cout << oss.str() << std::endl;
        // }

        // 创建输入文件
        static void createLogFile()
        {
            // 打开或创建文件
            f_stream = std::fstream(fileName, std::ios::out);
            if (f_stream.fail())
            {
                throw std::logic_error("open file failed " + fileName);
            }
            else
            {
                // std::cout << "open file success " << fileName << std::endl;
                is_openFile = true;
            }
        }

        // 初始化默认变量
        static void init_default()
        {
            colors = {
                LOG_DEFAULT, LOG_RED, LOG_GREEN, LOG_YELLOW, LOG_BLUE, LOG_MAGENTA, LOG_CYAN, LOG_WHITE, LOG_BLACK,
                LOG_BOLD, LOG_UNDERLINE, LOG_BLINK, LOG_INVERT, LOG_HIDDEN, LOG_CLEAR, LOG_WHITE_BG_GREEN, LOG_WHITE_BG_RED,
                LOG_WHITE_BG_YELLOW, LOG_WHITE_BG_BLUE, LOG_WHITE_BG_MAGENTA, LOG_WHITE_BG_CYAN, LOG_WHITE_BG_BLACK,
                LOG_BLACK_BG_WHITE, LOG_BLACK_BG_RED, LOG_BLACK_BG_YELLOW, LOG_BLACK_BG_BLUE, LOG_BLACK_BG_MAGENTA,
                LOG_BLACK_BG_CYAN};
            levels = {"DEBUG", "INFO ", "WARN ", "ERROR", "QUIET"};
            precision = 2;
            printLevelStr = levels[0];
            delimiter = " ";
            printColor = colors[int(LogColor::def)];
            fileName = "log.txt";
            printLevel = true;
            printTime = true;
            printToFile = false;
            is_openFile = false;

            is_init = true;
        }
    };
}

int sunray_logger::Logger::precision;             // 小数点精度
bool sunray_logger::Logger::printLevel;           // 是否打印level
bool sunray_logger::Logger::printTime;            // 是否打印时间
bool sunray_logger::Logger::printToFile;          // 是否输出到文件
bool sunray_logger::Logger::is_openFile;          // 是否打开了文件
bool sunray_logger::Logger::is_init;              // 是否初始化
std::string sunray_logger::Logger::delimiter{" "};     // 分隔符
std::string sunray_logger::Logger::printColor;    // 设置默认颜色
std::string sunray_logger::Logger::printLevelStr; // 设置默认level
std::string sunray_logger::Logger::fileName;      // 文件路径+文件名
std::fstream sunray_logger::Logger::f_stream;     // 文件输出流对象
std::array<std::string, 28> sunray_logger::Logger::colors;
std::array<std::string, 5> sunray_logger::Logger::levels;

#endif // PRINTF_FORMAT_H
