#include "WGS84.h"
#include "ros_msg_utils.h"

bool first_init{false};
mavros_msgs::GPSRAW gps_raw;
mavros_msgs::GPSRAW gps_raw_init;

// origin是原点的经纬度，target是目标点的经纬度
// 输入经纬度单位为度，高度单位为米
// 经度范围：-180°到180°（东经为正，西经为负）
// 纬度范围：-90°到90°（北纬为正，南纬为负）
LLH_Coord origin, target;
// result是转换得到的ENU坐标系坐标（原点为origin，ENU为方向）
ENU_Coord result;

// 无人机gps原始数据回调函数
void px4_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    if(!first_init)
    {
        first_init = true;
        gps_raw_init = *msg;
        Logger::info("first_init");
    }

    gps_raw = *msg;
}

void debug()
{

    Logger::print_color(int(LogColor::green), "原点GPS经纬高:",
                        (double)gps_raw_init.lat,
                        (double)gps_raw_init.lon,
                        "[deg*1e7]",
                        (double)gps_raw_init.alt/10e3,
                        "[m]");

    Logger::print_color(int(LogColor::green), "当前GPS经纬高:",
                        (double)gps_raw_init.lat,
                        (double)gps_raw_init.lon,
                        "[deg*1e7]",
                        (double)gps_raw_init.alt/10e3,
                        "[m]");

    Logger::print_color(int(LogColor::green), "解算位置[X Y Z]:",
                        result.x,
                        result.y,
                        result.z,
                        "[ m ]");
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();

    ros::init(argc, argv, "WGS84_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);

    ros::Subscriber px4_gps_raw_sub = nh.subscribe<mavros_msgs::GPSRAW>("/uav1/mavros/gpsstatus/gps1/raw", 10, px4_gps_raw_callback);

    while(ros::ok())
    {
        ros::spinOnce();

        // 设置初始点坐标（北京天安门）
        origin.lat = (double)gps_raw_init.lat/1e7;
        origin.lon = (double)gps_raw_init.lon/1e7;
        origin.alt = (double)gps_raw_init.alt/1e3;

        // 设置目标点坐标（附近某个点）
        target.lat = (double)gps_raw.lat/1e7;
        target.lon = (double)gps_raw.lon/1e7;
        target.alt = (double)gps_raw.alt/1e3;

        // 计算ENU坐标
        calculate_enu_coordinates(&origin, &target, &result);
        
        // 计算附加信息
        // double distance, horizontal_dist, azimuth, elevation;
        // calculate_enu_metrics(&result, &distance, &horizontal_dist, &azimuth, &elevation);
        // 输出结果
        // printf("ENU坐标结果:\n");
        // printf("东方向(X): %.3f m\n", result.x);
        // printf("北方向(Y): %.3f m\n", result.y);
        // printf("天方向(Z): %.3f m\n", result.z);
        // printf("总距离: %.3f m\n", distance);
        // printf("水平距离: %.3f m\n", horizontal_dist);
        // printf("方位角: %.2f°\n", azimuth);
        // printf("仰角: %.2f°\n", elevation);

        debug();
        rate.sleep();
    }

    // 程序结束
    ros::shutdown();
}
