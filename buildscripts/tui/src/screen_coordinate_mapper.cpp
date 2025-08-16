#include "screen_coordinate_mapper.hpp"

namespace sunray_tui {

// Linus-style: Simple, direct, no complex coordinate mapping needed
// Our two-section layout doesn't need mouse support initially

void ScreenCoordinateMapper::rebuild_mapping(const std::vector<struct RenderItem>& render_items, 
                                            int build_button_y, 
                                            bool dialog_visible, 
                                            int dialog_ok_y) {
    // Stub - not needed for two-section layout initially
}

ElementInfo ScreenCoordinateMapper::get_element_at(int y) const {
    // Stub - return unknown element  
    return ElementInfo();
}

ElementInfo ScreenCoordinateMapper::find_nearest_interactive(int y, int tolerance) const {
    // Stub - return unknown element
    return ElementInfo();
}

bool ScreenCoordinateMapper::validate_mapping() const {
    // Stub - always valid
    return true;
}

void ScreenCoordinateMapper::auto_detect_terminal_offset() {
    // Stub - no offset detection needed
}

void ScreenCoordinateMapper::debug_print_mapping() const {
    // Stub - no debug output needed
}

} // namespace sunray_tui