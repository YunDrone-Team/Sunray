#include "tui_config.hpp"
#include "tui_types.hpp"
#ifdef HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace sunray_tui {

std::unique_ptr<ConfigData> load_config(const std::string &file_path) {
#ifndef HAVE_YAML_CPP
  throw std::runtime_error("yaml-cpp support not compiled");
#else
  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("Failed to load YAML file: " +
                             std::string(e.what()));
  }

  auto data = std::make_unique<ConfigData>();
  if (config["modules"]) {
    const auto &modules_node = config["modules"];
    data->modules.reserve(modules_node.size());
    for (const auto &item : modules_node) {
      Module module;
      module.name = item.first.as<std::string>();
      const auto &props = item.second;
      module.description = props["description"].as<std::string>("");
      module.source_path = props["source_path"].as<std::string>("");
      module.build_path = props["build_path"].as<std::string>("");
      if (props["dependencies"] && props["dependencies"].IsSequence()) {
        const auto &deps = props["dependencies"];
        module.dependencies.reserve(deps.size());
        for (const auto &dep : deps)
          module.dependencies.emplace_back(dep.as<std::string>());
      }
      if (props["conflicts_with"] && props["conflicts_with"].IsSequence()) {
        const auto &conflicts = props["conflicts_with"];
        module.conflicts_with.reserve(conflicts.size());
        for (const auto &conflict : conflicts)
          module.conflicts_with.emplace_back(conflict.as<std::string>());
      }
      data->modules.emplace_back(std::move(module));
    }
  }
  if (config["groups"]) {
    const auto &groups_node = config["groups"];
    data->groups.reserve(groups_node.size());
    for (const auto &item : groups_node) {
      ModuleGroup group;
      group.name = item.first.as<std::string>();
      const auto &props = item.second;
      group.description = props["description"].as<std::string>("");
      if (props["modules"] && props["modules"].IsSequence()) {
        const auto &modules = props["modules"];
        group.modules.reserve(modules.size());
        for (const auto &mod : modules)
          group.modules.emplace_back(mod.as<std::string>());
      }
      data->groups.emplace_back(std::move(group));
    }
  }
  return data;
#endif
}

std::unique_ptr<ConfigData>
ConfigData::load_from_file(const std::string &file_path) {
  return load_config(file_path);
}

} // namespace sunray_tui