#pragma once
#include <memory>
#include <string>

namespace sunray_tui {
struct ConfigData;
std::unique_ptr<ConfigData> load_config(const std::string &file_path);
} // namespace sunray_tui