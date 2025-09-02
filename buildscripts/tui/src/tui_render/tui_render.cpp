#include "tui_render.hpp"
#include "ftxui/component/animation.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/screen/terminal.hpp"
#include "tui_terminal.hpp"

using namespace ftxui;

namespace sunray_tui {

UIRenderer::UIRenderer(UIState &state) : state_(state) {}

int UIRenderer::run_with_build_callback(std::function<void()> build_callback) {
  build_callback_ = build_callback;

  TerminalGuard guard;

  auto component = create_component();

  auto screen = ScreenInteractive::Fullscreen();

  // 设置触发退出的回调，解决TUI退出延迟问题
  state_.trigger_exit_callback = [&screen]() {
    screen.PostEvent(Event::Custom);
  };

  // 立即检查构建请求，修复了延迟问题
  bool should_build = false;

  auto wrapped_component = CatchEvent(component, [&](Event event) -> bool {
    // 如果构建被请求，立即退出
    if (state_.build_requested) {
      state_.build_requested = false;
      should_build = true;
      screen.ExitLoopClosure()();
      return true;
    }
    return false;
  });

  screen.Loop(wrapped_component);

  // FTXUI循环退出后，检查是否需要执行构建
  if (should_build && build_callback_) {
    build_callback_(); // TUI已退出，现在执行构建
  }

  return 0;
}

} // namespace sunray_tui