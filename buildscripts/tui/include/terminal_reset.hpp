#pragma once

#include "terminal_core.hpp"

namespace sunray_tui {

/**
 * Linus风格终端重置：直接使用Terminal::reset()，避免不必要的包装
 * 
 * 使用方式：
 * - Terminal::reset()           // 默认：清屏+重置状态
 * - Terminal::reset(false)      // 保留屏幕内容，只重置状态
 * - Terminal::reset(true, true) // 清屏+重置状态+设备重置
 */

} // namespace sunray_tui

// 向后兼容：如果确实需要，可以使用Terminal::reset()的简化调用
namespace terminal {
    // 不再提供包装函数，直接使用 sunray_tui::Terminal::reset()
}