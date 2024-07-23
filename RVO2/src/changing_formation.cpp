#include "changing_formation.h"

const float RVO_TWO_PI = 6.28318530717958647692F;

changingFormation::changingFormation(const ros::NodeHandle &nh)
{
    nh_ = nh;
    simulator = new RVO::RVOSimulator();
    nh_.param<int>("uav_id", uav_id, 1);
    nh_.param<string>("uav_name", uav_name, "uav");
    topic_prefix = "/" + uav_name + std::to_string(uav_id);
    odomSub_ = nh_.subscribe(topic_prefix + "/mavros/local_position/odom", 1, &changingFormation::odomCB, this);
    mocapSub_ = nh_.subscribe("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/pose", 1, &changingFormation::mocapCB, this);
    gazeboSub_ = nh_.subscribe(topic_prefix + "/mavros/local_position/odom", 1, &changingFormation::odomCB, this);
}

void changingFormation::odomCB(const nav_msgs::OdometryConstPtr &odom)
{
    last_pose_stamp = ros::Time::now(); // 记录时间戳，防止超时
    pos_from_external = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
}

void changingFormation::mocapCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    last_pose_stamp = ros::Time::now(); // 记录时间戳，防止超时
    pos_from_external = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void changingFormation::gazeboCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    last_pose_stamp = ros::Time::now(); // 记录时间戳，防止超时
    pos_from_external = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void changingFormation::setupScenario(RVO::RVOSimulator *simulator_, std::vector<RVO::Vector2> &goals_)
{
    /* Specify the global time step of the simulation. */
    simulator_->setTimeStep(0.25F);

    /* Specify the default parameters for agents that are subsequently added. */
    simulator_->setAgentDefaults(15.0F, 10U, 10.0F, 10.0F, 1.5F, 2.0F);

    /* Add agents, specifying their start position, and store their goals_ on the
     * opposite side of the environment. */
    // for (std::size_t i = 0U; i < goals_.size(); ++i)
    // {
    //     simulator->addAgent(
    //         200.0F *
    //         RVO::Vector2(std::cos(static_cast<float>(i) * RVO_TWO_PI * 0.004F),
    //                      std::sin(static_cast<float>(i) * RVO_TWO_PI * 0.004F)));
    //     goals_.push_back(-simulator->getAgentPosition(i));
    // }
}

void changingFormation::setPreferredVelocities(RVO::RVOSimulator *simulator_, const std::vector<RVO::Vector2> &goals_)
{
/* Set the preferred velocity to be a vector of unit magnitude (speed) in the
 * direction of the goal. */
#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
    for (int i = 0; i < static_cast<int>(simulator_->getNumAgents()); ++i)
    {
        RVO::Vector2 goalVector = goals_[i] - simulator_->getAgentPosition(i);

        if (RVO::absSq(goalVector) > 1.0F)
        {
            goalVector = RVO::normalize(goalVector);
        }
        simulator_->setAgentPrefVelocity(i, goalVector);
    }
}

bool changingFormation::reachedGoal(RVO::RVOSimulator *simulator_, const std::vector<RVO::Vector2> &goals_)
{
    /* Check if all agents have reached their goals_. */
    for (std::size_t i = 0U; i < simulator_->getNumAgents(); ++i)
    {
        if (RVO::absSq(simulator_->getAgentPosition(i) - goals_[i]) >
            simulator_->getAgentRadius(i) * simulator_->getAgentRadius(i))
        {
            return false;
        }
    }
    return true;
}

void changingFormation::run(RVO::RVOSimulator *simulator_, const std::vector<RVO::Vector2> &goals_)
{
    
    while(!reachedGoal(simulator_, goals_)){
        simulator_->setAgentPosition(i,RVO2::Vector2());
        if((ros::Time::now() - last_pose_stamp).toSec()>0.5){
            break;
        }
    }
    
}