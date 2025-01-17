#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include "ExternalPosition.h"
#include "ExternalPosition/MocapExternalPosition.h"
#include "ExternalPosition/OdomExternalPosition.h"
#include "ExternalPosition/PoseExternalPosition.h"
#include "ExternalPosition/ViobotExternalPosition.h"
// 外部定位工厂类
class ConcreteFactory {
public:
    virtual std::shared_ptr<ExternalPosition> create(const std::string& type) = 0;
};

class ExternalPositioFactory : public ConcreteFactory {
public:
    std::shared_ptr<ExternalPosition> create(const std::string& type) override {
        if (type == "ODOM" || type == "GAZEBO") {
            return std::make_shared<OdomExternalPosition>();
        } else if (type == "POSE") {
            return std::make_shared<PoseExternalPosition>();
        } else if (type == "MOCAP") {
            return std::make_shared<MocapExternalPosition>();
        }else if (type == "VIOBOT") {
            return std::make_shared<ViobotExternalPosition>();
        }
        return nullptr;
    }
};
