#include "terminal_guard.hpp"

namespace sunray_tui {

// 静态成员定义
std::atomic<bool> TerminalGuard::initialized_{false};
std::atomic<bool> TerminalGuard::cleanup_in_progress_{false};
std::atomic<bool> TerminalGuard::in_build_mode_{false};

} // namespace sunray_tui