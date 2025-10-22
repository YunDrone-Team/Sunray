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
    // 模拟地面站发布MissionCMD指令
    ros::init(argc, argv, "missioncmd_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);

    // 模拟发布
    ros::Publisher mission_cmd_pub = nh.advertise<sunray_msgs::MissionCMD>("/sunray/mission_cmd", 10);
    
    int CMD = 0;
    int Formation = 0;
    double state_desired[4];

    // 原点坐标，精确到小数点后7位时有厘米级精度
    origin_point.lat = 47.3977421;
    origin_point.lon =  8.5455940;
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
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>missioncmd_pub<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "CMD: "
             << YELLOW << " 1 " << GREEN << "TAKEOFF,"
             << YELLOW << " 2 " << GREEN << "LAND,"
             << YELLOW << " 3 " << GREEN << "RETURN,"
             << YELLOW << " 4 " << GREEN << "MOVE in ENU," 
             << YELLOW << " 5 " << GREEN << "MOVE in WGS84," << TAIL << endl;
        cin >> CMD;

        switch (CMD)
        {
        case 1:
            mission_cmd.mission = sunray_msgs::MissionCMD::TAKEOFF;
            mission_cmd.origin_lat = origin_point.lat;
            mission_cmd.origin_lon = origin_point.lon;
            mission_cmd.origin_alt = origin_point.alt;
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
            cout << BLUE << "请输入期望的阵型, 1 for FORMATION1, 2 for FORMATION2, 3 for FORMATION3." << endl;
            cin >> Formation;
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
            if (Formation == 1)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION1;
            }else if (Formation == 2)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION2;
            }else if (Formation == 3)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION3;
            }else
            {
                mission_cmd.mission_formation = Formation;
            }
            mission_cmd.origin_lat = origin_point.lat;
            mission_cmd.origin_lon = origin_point.lon;
            mission_cmd.origin_alt = origin_point.alt;
            mission_cmd.leader_lat_ref = target_point.lat;
            mission_cmd.leader_lon_ref = target_point.lon;
            mission_cmd.leader_alt_ref = target_point.alt;
            mission_cmd_pub.publish(mission_cmd);
            break;
        case 5:
            cout << BLUE << "请输入期望的阵型, 1 for FORMATION1, 2 for FORMATION2, 3 for FORMATION3." << endl;
            cin >> Formation;
            cout << BLUE << "请输入领机期望的经纬高（注意需要到小数点后7位）" << endl;
            cout << BLUE << "desired_lat: --- [deg] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_lon: --- [deg]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_alt: --- [m]" << endl;
            cin >> state_desired[2];

            mission_cmd.mission = sunray_msgs::MissionCMD::MOVE;
            if (Formation == 1)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION1;
            }else if (Formation == 2)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION2;
            }else if (Formation == 3)
            {
                mission_cmd.mission_formation = sunray_msgs::MissionCMD::FORMATION3;
            }else
            {
                mission_cmd.mission_formation = Formation;
            }
            mission_cmd.origin_lat = origin_point.lat;
            mission_cmd.origin_lon = origin_point.lon;
            mission_cmd.origin_alt = origin_point.alt;
            mission_cmd.leader_lat_ref = state_desired[0];
            mission_cmd.leader_lon_ref = state_desired[1];
            mission_cmd.leader_alt_ref = state_desired[2];
            mission_cmd_pub.publish(mission_cmd);
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
