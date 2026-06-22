/*
本程序功能：
    1. 通过终端交互发布 NAVRL 目标点到 /move_base_simple/goal。
    2. 保留 Sunray Takeoff / Land / Hover / XyzPosYaw 控制指令发布功能。
    3. NAVRL_GOAL 只需要用户输入 x、y；XyzPosYaw 需要输入完整位置和偏航角。
*/

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace {
constexpr const char* kReset = "\033[0m";
constexpr const char* kDim = "\033[2m";
constexpr const char* kTitleBg = "\033[1;44;37m";
constexpr const char* kBlue = "\033[1;34m";
constexpr const char* kGreen = "\033[1;32m";
constexpr const char* kYellow = "\033[1;33m";
constexpr const char* kRed = "\033[1;31m";
constexpr const char* kCyan = "\033[1;36m";

void printTitle(const std::string& title) {
    std::cout << "\n"
              << kBlue
              << "==================== "
              << kTitleBg << " " << title << " " << kReset
              << kBlue
              << " ===================="
              << kReset << std::endl;
}

void printInfoLine(const std::string& label, const std::string& value) {
    std::cout << kGreen << " " << label << " " << kReset << value << std::endl;
}

void printPrompt(const std::string& text) {
    std::cout << kCyan << text << kReset;
    std::cout.flush();
}

void printWarnLine(const std::string& text) {
    std::cout << kYellow << "[提示] " << kReset << text << std::endl;
}

void printErrorLine(const std::string& text) {
    std::cout << kRed << "[错误] " << kReset << text << std::endl;
}

void printSuccessLine(const std::string& text) {
    std::cout << kGreen << "[已发布] " << kReset << text << std::endl;
}

double degToRad(const double deg) {
    return deg * M_PI / 180.0;
}

void fillYawQuaternion(geometry_msgs::Quaternion& q, const double yaw_rad) {
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_rad * 0.5);
    q.w = std::cos(yaw_rad * 0.5);
}

std::string commandLabel(const int selection) {
    switch (selection) {
    case 1:
        return "NavRL 目标点";
    case 2:
        return "起飞 TAKEOFF";
    case 3:
        return "降落 LAND";
    case 4:
        return "悬停 HOVER";
    case 5:
        return "Sunray 定点 XyzPosYaw";
    default:
        return "未知指令";
    }
}

void printMenu(const std::string& goal_topic,
               const std::string& control_topic,
               const std::string& frame_id) {
    printTitle("NavRL 终端控制");
    printInfoLine("坐标系", frame_id);
    printInfoLine("NavRL 目标话题", goal_topic);
    printInfoLine("Sunray 控制话题", control_topic);
    std::cout << kDim << " 说明：位置单位为 m；输入 yaw 使用 deg，发布到控制指令时自动转换为 rad。"
              << kReset << std::endl;
    std::cout << kDim << " 规则：Takeoff / Land / Hover / XyzPosYaw 都只发布一次。"
              << kReset << std::endl;

    std::cout << kBlue << "------------------------------------------------------------" << kReset << std::endl;
    std::cout << kCyan << " 1 " << kReset << commandLabel(1)
              << kDim << "    输入 x y，发布 /move_base_simple/goal" << kReset << std::endl;
    std::cout << kCyan << " 2 " << kReset << commandLabel(2) << std::endl;
    std::cout << kCyan << " 3 " << kReset << commandLabel(3) << std::endl;
    std::cout << kCyan << " 4 " << kReset << commandLabel(4) << std::endl;
    std::cout << kCyan << " 5 " << kReset << commandLabel(5)
              << kDim << "    输入 x y z yaw，发布 Sunray uav_control_cmd" << kReset << std::endl;
    std::cout << kCyan << " 0 " << kReset << "退出" << std::endl;
    std::cout << kBlue << "------------------------------------------------------------" << kReset << std::endl;
    printPrompt("请输入功能编号: ");
}

bool readInt(std::istream& input, int& value) {
    if (!(input >> value)) {
        if (input.eof()) {
            printWarnLine("输入流已关闭，退出终端控制。");
            return false;
        }
        input.clear();
        input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        printErrorLine("输入无效，请输入数字编号。");
        return false;
    }
    return true;
}

bool readXY(std::istream& input, const std::string& title, double& x, double& y) {
    std::cout << kYellow << "\n[" << title << "]" << kReset << std::endl;
    printPrompt("请输入目标点 x[m] y[m]: ");
    if (!(input >> x >> y)) {
        if (input.eof()) {
            printWarnLine("输入流已关闭，退出终端控制。");
            return false;
        }
        input.clear();
        input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        printErrorLine("坐标输入无效，请按格式输入两个数字，例如：3.0 2.5");
        return false;
    }
    return true;
}

bool readPoint(std::istream& input, const std::string& title, double& x, double& y, double& z, double& yaw_deg) {
    std::cout << kYellow << "\n[" << title << "]" << kReset << std::endl;
    printPrompt("请输入期望位置 x[m] y[m] z[m] yaw[deg]: ");
    if (!(input >> x >> y >> z >> yaw_deg)) {
        if (input.eof()) {
            printWarnLine("输入流已关闭，退出终端控制。");
            return false;
        }
        input.clear();
        input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        printErrorLine("定点输入无效，请按格式输入四个数字，例如：3.0 2.0 1.2 90");
        return false;
    }
    return true;
}

std::string formatNavrlGoalResult(const std::string& topic,
                                  const std::string& frame_id,
                                  const double x,
                                  const double y) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(2);
    ss << "NavRL 目标点 -> topic: " << topic
       << " | frame: " << frame_id
       << " | x: " << x << " m"
       << " | y: " << y << " m";
    return ss.str();
}

std::string formatMovePointResult(const std::string& topic,
                                  const std::string& frame_id,
                                  const double x,
                                  const double y,
                                  const double z,
                                  const double yaw_deg,
                                  const double yaw_rad) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(2);
    ss << "Sunray 定点 -> topic: " << topic
       << " | frame: " << frame_id
       << " | x: " << x << " m"
       << " | y: " << y << " m"
       << " | z: " << z << " m"
       << " | yaw: " << yaw_deg << " deg / ";
    ss.precision(3);
    ss << yaw_rad << " rad";
    return ss.str();
}

sunray_msgs::UAVControlCMD makeBaseControlCmd(const uint8_t control_cmd, const std::string& frame_id) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = frame_id;
    cmd.cmd = control_cmd;
    cmd.desired_yaw = 0.0f;
    cmd.desired_yaw_rate = 0.0f;
    return cmd;
}

geometry_msgs::PoseStamped makeGoal(const double x,
                                    const double y,
                                    const double z,
                                    const double yaw_rad,
                                    const std::string& frame_id) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = frame_id;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    fillYawQuaternion(goal.pose.orientation, yaw_rad);
    return goal;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "navrl_terminal_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int uav_id = 1;
    std::string agent_name = "uav";
    std::string goal_topic = "/move_base_simple/goal";
    std::string control_topic;
    std::string frame_id = "map";

    private_nh.param("uav_id", uav_id, uav_id);
    private_nh.param<std::string>("agent_name", agent_name, agent_name);
    private_nh.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_nh.param<std::string>("frame_id", frame_id, frame_id);

    control_topic = "/" + agent_name + std::to_string(uav_id) + "/sunray/uav_control_cmd";
    private_nh.param<std::string>("control_topic", control_topic, control_topic);

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1, true);
    ros::Publisher control_pub = nh.advertise<sunray_msgs::UAVControlCMD>(control_topic, 1);
    ros::Duration(0.5).sleep();

    std::istream* input = &std::cin;

    printTitle("NavRL 终端控制已启动");
    printInfoLine("NavRL 目标话题", goal_topic);
    printInfoLine("Sunray 控制话题", control_topic);
    printInfoLine("坐标系", frame_id);

    while (ros::ok()) {
        printMenu(goal_topic, control_topic, frame_id);

        int selection = -1;
        if (!readInt(*input, selection)) {
            if (input->eof()) {
                break;
            }
            continue;
        }

        if (selection == 0) {
            printWarnLine("退出 NavRL 终端控制。");
            break;
        }

        if (selection == 1) {
            double x = 0.0;
            double y = 0.0;
            if (!readXY(*input, "NAVRL_GOAL", x, y)) {
                if (input->eof()) {
                    break;
                }
                continue;
            }

            const geometry_msgs::PoseStamped goal = makeGoal(x, y, 0.0, 0.0, frame_id);
            goal_pub.publish(goal);
            printSuccessLine(formatNavrlGoalResult(goal_topic, frame_id, x, y));
        } else if (selection == 2 || selection == 3 || selection == 4) {
            uint8_t control_cmd = sunray_msgs::UAVControlCMD::Hover;
            std::string command_name;

            if (selection == 2) {
                control_cmd = sunray_msgs::UAVControlCMD::Takeoff;
                command_name = "Takeoff";
            } else if (selection == 3) {
                control_cmd = sunray_msgs::UAVControlCMD::Land;
                command_name = "Land";
            } else {
                control_cmd = sunray_msgs::UAVControlCMD::Hover;
                command_name = "Hover";
            }

            sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(control_cmd, frame_id);
            control_pub.publish(cmd);
            printSuccessLine(commandLabel(selection) + " -> topic: " + control_topic);
        } else if (selection == 5) {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double yaw_deg = 0.0;
            if (!readPoint(*input, "XyzPosYaw", x, y, z, yaw_deg)) {
                if (input->eof()) {
                    break;
                }
                continue;
            }

            const double yaw_rad = degToRad(yaw_deg);
            sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::XyzPosYaw, frame_id);
            cmd.desired_pos[0] = static_cast<float>(x);
            cmd.desired_pos[1] = static_cast<float>(y);
            cmd.desired_pos[2] = static_cast<float>(z);
            cmd.desired_yaw = static_cast<float>(yaw_rad);
            control_pub.publish(cmd);

            printSuccessLine(formatMovePointResult(control_topic, frame_id, x, y, z, yaw_deg, yaw_rad));
        } else {
            printWarnLine("未知功能编号：" + std::to_string(selection));
        }

        ros::spinOnce();
    }

    return 0;
}
