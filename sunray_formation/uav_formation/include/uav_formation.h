#include <ros/ros.h>
#include "RVOSimulator.h"
#include "Vector2.h"

// neighborDist: 10.0
// maxNeighbors: 10
// timeHorizon: 10.0
// timeHorizonObst: 3.0
// radius: 0.08
// maxSpeed: 1
// timestep: 0.2 # 1 / controller rate
// axel_width: 0.255
// los_margin: 0.1

// waypoint_spacing: 0.1
// path_margin: 0.6
// goal_tolerance: 0.1

class UAV_formation
{
private:
    RVO::RVOSimulator *sim;
    RVO::Vector2 goal;
    std::vector<RVO::Vector2> waypoints;

    float max_speed;
    float neighbor_dist;
    float time_horizon;
    float time_horizon_obst;
    float radius;
    float goal_radius;
    float waypoint_spacing;
    float path_margin;
    float goal_tolerance;
    float axel_width;
    float los_margin;
public:
    UAV_formation(/* args */);
    ~UAV_formation();
};

UAV_formation::UAV_formation(/* args */)
{
}

UAV_formation::~UAV_formation()
{
}
