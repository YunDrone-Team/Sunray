#include "screen_coordinate_mapper.hpp"
#include "ui_core.hpp"
#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <cstdlib>

namespace sunray_tui {

void ScreenCoordinateMapper::rebuild_mapping(const std::vector<RenderItem>& render_items, 
                                            int build_button_y, 
                                            bool dialog_shown,
                                            int dialog_ok_button_y) {
    coordinate_map.clear();
    
    // 计算基础偏移量：border(1) + title(1) + separator(1) = 3
    const int content_start_y = 3 + base_offset;
    
    // 映射render_items到屏幕坐标
    for (size_t i = 0; i < render_items.size(); ++i) {
        const auto& item = render_items[i];
        int screen_y = content_start_y + static_cast<int>(i);
        
        ElementType element_type = ElementType::UNKNOWN;
        switch (item.type) {
            case RenderItem::GROUP_HEADER:
                element_type = ElementType::GROUP_HEADER;
                break;
            case RenderItem::MODULE_ITEM:
                element_type = ElementType::MODULE_ITEM;
                break;
            case RenderItem::SEPARATOR:
                element_type = ElementType::SEPARATOR;
                break;
            case RenderItem::INFO_TEXT:
                element_type = ElementType::INFO_TEXT;
                break;
        }
        
        ElementInfo element_info(element_type, static_cast<int>(i), item.identifier);
        
        // 禁用的模块不可点击
        if (element_type == ElementType::MODULE_ITEM && item.is_disabled) {
            element_info.is_clickable = false;
            element_info.is_hoverable = false;
        }
        
        coordinate_map.emplace_back(screen_y, element_info);
    }
    
    // 添加编译按钮
    if (build_button_y >= 0 && !dialog_shown) {
        ElementInfo button_info(ElementType::BUILD_BUTTON, -1, "build_button");
        coordinate_map.emplace_back(build_button_y, button_info);
    }
    
    // 添加对话框OK按钮
    if (dialog_shown && dialog_ok_button_y >= 0) {
        ElementInfo ok_button_info(ElementType::DIALOG_OK_BUTTON, -1, "dialog_ok");
        coordinate_map.emplace_back(dialog_ok_button_y, ok_button_info);
    }
    
    // 按screen_y排序，便于查找
    std::sort(coordinate_map.begin(), coordinate_map.end(), 
              [](const ScreenElement& a, const ScreenElement& b) {
                  return a.screen_y < b.screen_y;
              });
}

ElementInfo ScreenCoordinateMapper::get_element_at(int screen_y) const {
    // 精确匹配
    for (const auto& element : coordinate_map) {
        if (element.screen_y == screen_y) {
            return element.element_info;
        }
    }
    
    // 返回默认的未知元素
    return ElementInfo();
}

bool ScreenCoordinateMapper::is_clickable_at(int screen_y) const {
    ElementInfo info = get_element_at(screen_y);
    return info.is_clickable;
}

bool ScreenCoordinateMapper::is_hoverable_at(int screen_y) const {
    ElementInfo info = get_element_at(screen_y);
    return info.is_hoverable;
}

ElementInfo ScreenCoordinateMapper::find_nearest_interactive(int screen_y, int max_distance) const {
    // 首先尝试精确匹配
    ElementInfo exact = get_element_at(screen_y);
    if (exact.is_clickable || exact.is_hoverable) {
        return exact;
    }
    
    // 在指定范围内查找最近的可交互元素
    ElementInfo best_match;
    int best_distance = max_distance + 1;
    
    for (const auto& element : coordinate_map) {
        if (!element.element_info.is_clickable && !element.element_info.is_hoverable) {
            continue;
        }
        
        int distance = std::abs(element.screen_y - screen_y);
        if (distance <= max_distance && distance < best_distance) {
            best_match = element.element_info;
            best_distance = distance;
        }
    }
    
    return best_match;
}

void ScreenCoordinateMapper::debug_print_mapping() const {
    std::cout << "=== Screen Coordinate Mapping Debug ===" << std::endl;
    std::cout << "Base offset: " << base_offset << std::endl;
    
    for (const auto& element : coordinate_map) {
        const char* type_name = "UNKNOWN";
        switch (element.element_info.type) {
            case ElementType::GROUP_HEADER: type_name = "GROUP_HEADER"; break;
            case ElementType::MODULE_ITEM: type_name = "MODULE_ITEM"; break;
            case ElementType::SEPARATOR: type_name = "SEPARATOR"; break;
            case ElementType::INFO_TEXT: type_name = "INFO_TEXT"; break;
            case ElementType::BUILD_BUTTON: type_name = "BUILD_BUTTON"; break;
            case ElementType::DIALOG_OK_BUTTON: type_name = "DIALOG_OK_BUTTON"; break;
            default: break;
        }
        
        std::cout << "Y=" << element.screen_y 
                  << " Type=" << type_name
                  << " Index=" << element.element_info.render_item_index
                  << " ID=" << element.element_info.identifier
                  << " Clickable=" << (element.element_info.is_clickable ? "YES" : "NO")
                  << " Hoverable=" << (element.element_info.is_hoverable ? "YES" : "NO")
                  << std::endl;
    }
    std::cout << "==========================================" << std::endl;
}

void ScreenCoordinateMapper::debug_mouse_position(int mouse_y) const {
    std::cout << "=== Mouse Position Debug ===" << std::endl;
    std::cout << "Mouse Y: " << mouse_y << std::endl;
    
    // 精确匹配
    ElementInfo exact = get_element_at(mouse_y);
    if (exact.type != ElementType::UNKNOWN) {
        std::cout << "Exact match found!" << std::endl;
    } else {
        std::cout << "No exact match, searching nearby..." << std::endl;
        
        // 查找附近元素
        for (int distance = 1; distance <= 3; ++distance) {
            ElementInfo above = get_element_at(mouse_y - distance);
            ElementInfo below = get_element_at(mouse_y + distance);
            
            if (above.type != ElementType::UNKNOWN) {
                std::cout << "Found element " << distance << " lines above" << std::endl;
            }
            if (below.type != ElementType::UNKNOWN) {
                std::cout << "Found element " << distance << " lines below" << std::endl;
            }
        }
    }
    std::cout << "=========================" << std::endl;
}

void ScreenCoordinateMapper::auto_detect_terminal_offset() {
    // 简单的终端检测：尝试不同的偏移量
    // 在实际使用中，这可以通过检测第一个有效点击来自动校正
    
    // 检测环境变量中的终端类型
    const char* term = std::getenv("TERM");
    const char* term_program = std::getenv("TERM_PROGRAM");
    
    if (term_program && std::string(term_program) == "Apple_Terminal") {
        // macOS Terminal可能需要不同的偏移
        base_offset = 0;
    } else if (term && std::string(term).find("xterm") != std::string::npos) {
        // xterm系列
        base_offset = 0;
    } else if (term && std::string(term).find("screen") != std::string::npos) {
        // screen/tmux
        base_offset = 0;
    } else {
        // 默认值
        base_offset = 0;
    }
}

bool ScreenCoordinateMapper::validate_mapping() const {
    if (coordinate_map.empty()) {
        return false;
    }
    
    // 检查坐标是否按顺序排列
    for (size_t i = 1; i < coordinate_map.size(); ++i) {
        if (coordinate_map[i].screen_y <= coordinate_map[i-1].screen_y) {
            std::cout << "Warning: Coordinate mapping not properly sorted!" << std::endl;
            return false;
        }
    }
    
    // 检查是否有重复的坐标
    std::unordered_set<int> seen_coords;
    for (const auto& element : coordinate_map) {
        if (seen_coords.count(element.screen_y)) {
            std::cout << "Warning: Duplicate screen coordinates found: Y=" << element.screen_y << std::endl;
            return false;
        }
        seen_coords.insert(element.screen_y);
    }
    
    return true;
}

} // namespace sunray_tui