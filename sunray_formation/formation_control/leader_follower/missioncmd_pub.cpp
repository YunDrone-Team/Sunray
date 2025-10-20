#include "ros_msg_utils.h"

using namespace std;

sunray_msgs::MissionCMD mission_cmd;

LLH_Coord origin_point;
LLH_Coord target_point;
ENU_Coord target_point_xyz;
void mySigintHandler(int sig)
{
    ROS_INFO("[missioncmd_pub] exit...");
    ros::shutdown();
    exit(0);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "missioncmd_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);
    int uav_id;
    int uav_num;
    string uav_name{""};
    nh.param("uav_id", uav_id, 1);
    nh.param("uav_num", uav_num, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = uav_name + std::to_string(uav_id);
    ros::Publisher mission_cmd_pub = nh.advertise<sunray_msgs::MissionCMD>("/sunray/mission_cmd", 10);
    int CMD = 0;
    float state_desired[4];

    origin_point.lat = 47.397742;
    origin_point.lon = 8.5455932;
    origin_point.alt = 535.7399167266602;


    mission_cmd.mission = sunray_msgs::MissionCMD::LAND;
    mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION1;

    mission_cmd.origin_lat = origin_point.lat;
    mission_cmd.origin_lon = origin_point.lon;
    mission_cmd.origin_alt = origin_point.alt;

    mission_cmd.leader_lat_ref = origin_point.lat;
    mission_cmd.leader_lon_ref = origin_point.lon;
    mission_cmd.leader_alt_ref = origin_point.alt;


    while (ros::ok())
    {
        ros::spinOnce();
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>mission_cmd<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "CMD: "
             << YELLOW << " 1 " << GREEN << "TAKEOFF,"
             << YELLOW << " 2 " << GREEN << "LAND,"
             << YELLOW << " 3 " << GREEN << "RETURN,"
             << YELLOW << " 4 " << GREEN << "MOVE," << TAIL << endl;
        cin >> CMD;

        switch (CMD)
        {
        case 1:
            mission_cmd.mission = sunray_msgs::MissionCMD::TAKEOFF;
            mission_cmd_pub.publish(mission_cmd);
            break;
        case 2:
            mission_cmd.mission = sunray_msgs::MissionCMD::LAND;
            mission_cmd_pub.publish(mission_cmd);
            break;
        case 3:
            mission_cmd.mission = sunray_msgs::MissionCMD::RETURN;
            mission_cmd_pub.publish(mission_cmd);
            break;
        case 4:
            cout << BLUE << "请输入领机期望的[X Y Z]" << endl;
            cout << BLUE << "desired_pos: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_pos: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_pos: --- z [m]" << endl;
            cin >> state_desired[2];

            target_point_xyz.x = state_desired[0];
            target_point_xyz.y = state_desired[1];
            target_point_xyz.z = state_desired[2];

            target_point = enu_to_llh(&origin_point, &target_point_xyz);

            mission_cmd.mission = sunray_msgs::MissionCMD::MOVE;
            mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION1;

            mission_cmd.origin_lat = origin_point.lat;
            mission_cmd.origin_lon = origin_point.lon;
            mission_cmd.origin_alt = origin_point.alt;

            mission_cmd.leader_lat_ref = target_point.lat;
            mission_cmd.leader_lon_ref = target_point.lon;
            mission_cmd.leader_alt_ref = target_point.alt;

            mission_cmd_pub.publish(mission_cmd);
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
