#pragma once

#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <unistd.h>


namespace sunray_tui {

/**
 * Linus风格终端核心：统一的底层实现，消除代码重复
 * 职责：终端ANSI序列管理 + TTY直接操作
 */
class Terminal {
public:
  /**
   * 核心重置方法：统一的终端状态重置
   * @param clear_screen 是否清除屏幕内容
   * @param include_device_reset 是否包含设备重置序列
   */
  static void reset(bool clear_screen = true,
                    bool include_device_reset = false) {
    const char *reset_sequence =
        get_reset_sequence(clear_screen, include_device_reset);
    write_sequence(reset_sequence);
  }

  /**
   * TUI无缝退出：只重置终端模式，保持屏幕内容和历史记录完全不变
   */
  static void seamless_tui_exit() {
    // 精确控制：只重置必要的终端模式，完全不触碰屏幕缓冲区
    static const char *const SEAMLESS_EXIT_SEQUENCE =
        "\033[?25h"   // Show cursor
        "\033[0m"     // Reset all attributes (colors, styles)
        "\033[?1000l" // Disable X10 mouse reporting
        "\033[?1001l" // Disable highlight mouse tracking
        "\033[?1002l" // Disable button event tracking
        "\033[?1003l" // Disable any event tracking
        "\033[?1004l" // Disable focus events
        "\033[?1005l" // Disable UTF-8 mouse mode
        "\033[?1006l" // Disable SGR mouse mode
        "\033[?1007l" // Disable alternate scroll mode
        "\033[?1015l" // Disable Urxvt mouse mode
        "\033[?1016l" // Disable SGR-Pixels mode
        "\033[?2004l" // Disable bracketed paste mode
        "\033[?1049l" // Exit alternate screen + restore cursor
                      // (关键：只这一个屏幕操作)
        "\033[?12l"   // Disable cursor blinking
        "\033[?7h"    // Enable auto-wrap mode
        "\033[?8h";   // Enable auto-repeat mode

    write_sequence(SEAMLESS_EXIT_SEQUENCE);
  }

  /**
   * TTY直接写入：避免stdout污染，增强缓冲区清理
   */
  static void write_sequence(const char *sequence) {
    if (!sequence)
      return;

    // Write directly to TTY to avoid stdout pollution
    int tty_fd = open("/dev/tty", O_WRONLY);
    if (tty_fd >= 0) {
      write(tty_fd, sequence, strlen(sequence));
      fsync(tty_fd);
      close(tty_fd);

      // Linus风格：增强延迟，确保终端完全处理控制序列
      usleep(150000); // 0.15秒，给终端更多时间处理
    } else {
      // Fallback to stdout if TTY unavailable
      std::cout << sequence << std::flush;
      usleep(150000); // 确保stdout也有足够处理时间
    }

    // 额外的缓冲区清理：尝试丢弃任何待处理的终端响应
    flush_terminal_input();
  }

private:
  /**
   * 清理终端输入缓冲区：丢弃残留的控制字符
   */
  static void flush_terminal_input() {
    int tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (tty_fd >= 0) {
      char buffer[256];
      // 非阻塞读取，丢弃所有待处理的输入
      while (read(tty_fd, buffer, sizeof(buffer)) > 0) {
        // 继续读取直到缓冲区清空
      }
      close(tty_fd);
    }
  }
  /**
   * 生成ANSI重置序列：根据参数组合不同的重置级别
   */
  static const char *get_reset_sequence(bool clear_screen,
                                        bool include_device_reset) {
    // Linus风格：静态常量，避免重复分配
    static const char *const RESET_PRESERVE_CONTENT =
        "\033[?25h"   // Show cursor
        "\033[0m"     // Reset all attributes (colors, styles)
        "\033[?1000l" // Disable X10 mouse reporting
        "\033[?1001l" // Disable highlight mouse tracking
        "\033[?1002l" // Disable button event tracking
        "\033[?1003l" // Disable any event tracking
        "\033[?1004l" // Disable focus events
        "\033[?1005l" // Disable UTF-8 mouse mode
        "\033[?1006l" // Disable SGR mouse mode (重要：修复^[[<M问题)
        "\033[?1007l" // Disable alternate scroll mode
        "\033[?1015l" // Disable Urxvt mouse mode
        "\033[?1016l" // Disable SGR-Pixels mode
        "\033[?2004l" // Disable bracketed paste mode
        "\033[?47l"   // Exit alternate screen mode
        "\033[?1049l" // Exit alternate screen + cursor save
        "\033[?12l"   // Disable cursor blinking
        "\033[?7h"    // Enable auto-wrap mode
        "\033[?8h";   // Enable auto-repeat mode

    static const char *const RESET_CLEAR_SCREEN =
        "\033[?25h"   // Show cursor
        "\033[0m"     // Reset all attributes (colors, styles)
        "\033[2J"     // Clear entire screen
        "\033[H"      // Move cursor to home (1,1)
        "\033[?1000l" // Disable X10 mouse reporting
        "\033[?1001l" // Disable highlight mouse tracking
        "\033[?1002l" // Disable button event tracking
        "\033[?1003l" // Disable any event tracking
        "\033[?1004l" // Disable focus events
        "\033[?1005l" // Disable UTF-8 mouse mode
        "\033[?1006l" // Disable SGR mouse mode (重要：修复^[[<M问题)
        "\033[?1007l" // Disable alternate scroll mode
        "\033[?1015l" // Disable Urxvt mouse mode
        "\033[?1016l" // Disable SGR-Pixels mode
        "\033[?2004l" // Disable bracketed paste mode
        "\033[?47l"   // Exit alternate screen mode
        "\033[?1049l" // Exit alternate screen + cursor save
        "\033[?12l"   // Disable cursor blinking
        "\033[?7h"    // Enable auto-wrap mode
        "\033[?8h";   // Enable auto-repeat mode

    static const char *const RESET_WITH_DEVICE =
        "\033[?25h"   // Show cursor
        "\033[0m"     // Reset all attributes (colors, styles)
        "\033[2J"     // Clear entire screen
        "\033[H"      // Move cursor to home (1,1)
        "\033[?1000l" // Disable X10 mouse reporting
        "\033[?1001l" // Disable highlight mouse tracking
        "\033[?1002l" // Disable button event tracking
        "\033[?1003l" // Disable any event tracking
        "\033[?1004l" // Disable focus events
        "\033[?1005l" // Disable UTF-8 mouse mode
        "\033[?1006l" // Disable SGR mouse mode (重要：修复^[[<M问题)
        "\033[?1007l" // Disable alternate scroll mode
        "\033[?1015l" // Disable Urxvt mouse mode
        "\033[?1016l" // Disable SGR-Pixels mode
        "\033[?2004l" // Disable bracketed paste mode
        "\033[?47l"   // Exit alternate screen mode
        "\033[?1049l" // Exit alternate screen + cursor save
        "\033[?12l"   // Disable cursor blinking
        "\033[?7h"    // Enable auto-wrap mode
        "\033[?8h";   // Enable auto-repeat mode
                      // 注意：移除"\033c"设备重置，因为它会触发^[[?1;2c查询响应

    // 根据参数选择合适的序列
    if (include_device_reset) {
      return RESET_WITH_DEVICE;
    } else if (clear_screen) {
      return RESET_CLEAR_SCREEN;
    } else {
      return RESET_PRESERVE_CONTENT;  // 修复：返回保留内容的序列，不是0
    }
  }
};

} // namespace sunray_tui