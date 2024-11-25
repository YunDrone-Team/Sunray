#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "VsionMavros.h"

class ExternalPosition
{
public:
    ExternalPosition()
    {
    }

    struct ExternalPositionState
    {
        double px;
        double py;
        double pz;
        double vx;
        double vy;
        double vz;
        double qx;
        double qy;
        double qz;
        double qw;
        double roll;
        double pitch;
        double yaw;
        double timeout_count;
        bool valid;
    };

    ExternalPositionState position_state;
    VisionPosition vision_position;
    VsionMavros external_mavros;

    virtual void init(int id, std::string name, std::string souce_topic) = 0;
    // 实现定时回调函数
    virtual void timerCallback(const ros::TimerEvent &event) = 0;

    virtual void bindTopic(ros::NodeHandle &nh) = 0;

private:
    
    
};

#endif // EXTERNALPOSITION_H