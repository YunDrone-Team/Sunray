/*
本程序功能：
    1. 通过终端交互控制 FUEL 探索启动、暂停、恢复、停止。
    2. 保留 Sunray Takeoff / Land / Hover 控制指令发布功能。
    3. Land 会先停止 FUEL 探索，再发布降落指令，避免轨迹指令和降落指令竞争。
*/

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVSetup.h>
#include <sunray_msgs/UAVState.h>

#include <iostream>
#include <limits>
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

std::string g_fuel_state = "UNKNOWN";
sunray_msgs::UAVState g_uav_state;
bool g_have_uav_state = false;

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

std::string commandLabel(const int selection) {
  switch (selection) {
    case 1:
      return "启动 FUEL 探索";
    case 2:
      return "暂停 FUEL 探索";
    case 3:
      return "恢复 FUEL 探索";
    case 4:
      return "停止 FUEL 探索";
    case 5:
      return "起飞 TAKEOFF";
    case 6:
      return "悬停 HOVER";
    case 7:
      return "降落 LAND";
    default:
      return "未知指令";
  }
}

void printMenu(const std::string& start_topic,
               const std::string& pause_topic,
               const std::string& stop_topic,
               const std::string& setup_topic,
               const std::string& control_topic,
               const std::string& uav_state_topic,
               const std::string& state_topic,
               const std::string& frame_id) {
  printTitle("FUEL 终端控制");
  printInfoLine("坐标系", frame_id);
  printInfoLine("FUEL 状态话题", state_topic);
  printInfoLine("FUEL 当前状态", g_fuel_state);
  printInfoLine("Sunray 设置话题", setup_topic);
  printInfoLine("Sunray 控制话题", control_topic);
  printInfoLine("Sunray 状态话题", uav_state_topic);
  std::cout << kDim << " 规则：FUEL 控制只影响探索状态；Takeoff / Hover / Land 发布 Sunray 控制指令。"
            << kReset << std::endl;
  std::cout << kDim << " 规则：Land 会先停止 FUEL 探索，再发布降落指令。"
            << kReset << std::endl;

  std::cout << kBlue << "------------------------------------------------------------" << kReset << std::endl;
  std::cout << kCyan << " 1 " << kReset << commandLabel(1)
            << kDim << "    发布 " << start_topic << kReset << std::endl;
  std::cout << kCyan << " 2 " << kReset << commandLabel(2)
            << kDim << "    发布 " << pause_topic << " = true" << kReset << std::endl;
  std::cout << kCyan << " 3 " << kReset << commandLabel(3)
            << kDim << "    发布 " << pause_topic << " = false" << kReset << std::endl;
  std::cout << kCyan << " 4 " << kReset << commandLabel(4)
            << kDim << "    发布 " << stop_topic << kReset << std::endl;
  std::cout << kCyan << " 5 " << kReset << commandLabel(5) << std::endl;
  std::cout << kCyan << " 6 " << kReset << commandLabel(6) << std::endl;
  std::cout << kCyan << " 7 " << kReset << commandLabel(7) << std::endl;
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

sunray_msgs::UAVControlCMD makeBaseControlCmd(const uint8_t control_cmd, const std::string& frame_id) {
  sunray_msgs::UAVControlCMD cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = frame_id;
  cmd.cmd = control_cmd;
  cmd.desired_yaw = 0.0f;
  cmd.desired_yaw_rate = 0.0f;
  return cmd;
}

void fuelStateCallback(const std_msgs::StringConstPtr& msg) {
  g_fuel_state = msg->data;
}

void uavStateCallback(const sunray_msgs::UAVStateConstPtr& msg) {
  g_uav_state = *msg;
  g_have_uav_state = true;
}

void publishStart(const ros::Publisher& start_pub, const std::string& topic) {
  start_pub.publish(std_msgs::Empty());
  printSuccessLine("启动 FUEL 探索 -> topic: " + topic);
}

void publishStop(const ros::Publisher& stop_pub, const std::string& topic) {
  stop_pub.publish(std_msgs::Empty());
  printSuccessLine("停止 FUEL 探索 -> topic: " + topic);
}

void publishPause(const ros::Publisher& pause_pub, const std::string& topic, const bool paused) {
  std_msgs::Bool msg;
  msg.data = paused;
  pause_pub.publish(msg);
  printSuccessLine((paused ? "暂停 FUEL 探索 -> topic: " : "恢复 FUEL 探索 -> topic: ") + topic);
}

void publishControlMode(const ros::Publisher& setup_pub,
                        const std::string& setup_topic,
                        const std::string& control_mode) {
  sunray_msgs::UAVSetup setup;
  setup.header.stamp = ros::Time::now();
  setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
  setup.control_mode = control_mode;
  setup_pub.publish(setup);
  printSuccessLine("SET_CONTROL_MODE " + control_mode + " -> topic: " + setup_topic);
}

void publishArm(const ros::Publisher& setup_pub, const std::string& setup_topic) {
  sunray_msgs::UAVSetup setup;
  setup.header.stamp = ros::Time::now();
  setup.cmd = sunray_msgs::UAVSetup::ARM;
  setup_pub.publish(setup);
  printSuccessLine("ARM -> topic: " + setup_topic);
}

void publishControl(const ros::Publisher& control_pub,
                    const std::string& control_topic,
                    const std::string& frame_id,
                    const uint8_t control_cmd,
                    const std::string& command_name) {
  const sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(control_cmd, frame_id);
  control_pub.publish(cmd);
  printSuccessLine(command_name + " -> topic: " + control_topic);
}

void publishTakeoffSequence(const ros::Publisher& setup_pub,
                            const ros::Publisher& control_pub,
                            const std::string& setup_topic,
                            const std::string& control_topic,
                            const std::string& frame_id) {
  publishControlMode(setup_pub, setup_topic, "CMD_CONTROL");
  ros::Duration(0.3).sleep();
  ros::spinOnce();

  publishArm(setup_pub, setup_topic);
  ros::Duration(0.8).sleep();
  ros::spinOnce();

  if (g_have_uav_state && !g_uav_state.armed) {
    printWarnLine("当前 UAV 状态仍未解锁，Takeoff 可能不会被执行。");
  }

  for (int i = 0; ros::ok() && i < 3; ++i) {
    publishControl(control_pub, control_topic, frame_id, sunray_msgs::UAVControlCMD::Takeoff, "Takeoff");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_terminal_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int uav_id = 1;
  std::string agent_name = "uav";
  std::string start_topic = "/fuel/start_exploration";
  std::string pause_topic = "/fuel/pause_exploration";
  std::string stop_topic = "/fuel/stop_exploration";
  std::string state_topic = "/fuel/exploration_state";
  std::string setup_topic;
  std::string control_topic;
  std::string uav_state_topic;
  std::string frame_id = "world";

  private_nh.param("uav_id", uav_id, uav_id);
  private_nh.param<std::string>("agent_name", agent_name, agent_name);
  private_nh.param<std::string>("start_topic", start_topic, start_topic);
  private_nh.param<std::string>("pause_topic", pause_topic, pause_topic);
  private_nh.param<std::string>("stop_topic", stop_topic, stop_topic);
  private_nh.param<std::string>("state_topic", state_topic, state_topic);
  private_nh.param<std::string>("frame_id", frame_id, frame_id);

  setup_topic = "/" + agent_name + std::to_string(uav_id) + "/sunray/setup";
  control_topic = "/" + agent_name + std::to_string(uav_id) + "/sunray/uav_control_cmd";
  uav_state_topic = "/" + agent_name + std::to_string(uav_id) + "/sunray/uav_state";
  private_nh.param<std::string>("setup_topic", setup_topic, setup_topic);
  private_nh.param<std::string>("control_topic", control_topic, control_topic);
  private_nh.param<std::string>("uav_state_topic", uav_state_topic, uav_state_topic);

  ros::Publisher start_pub = nh.advertise<std_msgs::Empty>(start_topic, 1);
  ros::Publisher pause_pub = nh.advertise<std_msgs::Bool>(pause_topic, 1);
  ros::Publisher stop_pub = nh.advertise<std_msgs::Empty>(stop_topic, 1);
  ros::Publisher setup_pub = nh.advertise<sunray_msgs::UAVSetup>(setup_topic, 1);
  ros::Publisher control_pub = nh.advertise<sunray_msgs::UAVControlCMD>(control_topic, 1);
  ros::Subscriber state_sub = nh.subscribe(state_topic, 1, fuelStateCallback);
  ros::Subscriber uav_state_sub = nh.subscribe(uav_state_topic, 1, uavStateCallback);
  ros::Duration(0.5).sleep();

  std::istream* input = &std::cin;

  printTitle("FUEL 终端控制已启动");
  printInfoLine("FUEL 启动话题", start_topic);
  printInfoLine("FUEL 暂停话题", pause_topic);
  printInfoLine("FUEL 停止话题", stop_topic);
  printInfoLine("FUEL 状态话题", state_topic);
  printInfoLine("Sunray 设置话题", setup_topic);
  printInfoLine("Sunray 控制话题", control_topic);
  printInfoLine("Sunray 状态话题", uav_state_topic);
  printInfoLine("坐标系", frame_id);

  while (ros::ok()) {
    ros::spinOnce();
    printMenu(start_topic, pause_topic, stop_topic, setup_topic, control_topic, uav_state_topic, state_topic,
              frame_id);

    int selection = -1;
    if (!readInt(*input, selection)) {
      if (input->eof()) {
        break;
      }
      continue;
    }

    if (selection == 0) {
      printWarnLine("退出 FUEL 终端控制。");
      break;
    }

    if (selection == 1) {
      publishStart(start_pub, start_topic);
    } else if (selection == 2) {
      publishPause(pause_pub, pause_topic, true);
    } else if (selection == 3) {
      publishPause(pause_pub, pause_topic, false);
    } else if (selection == 4) {
      publishStop(stop_pub, stop_topic);
    } else if (selection == 5) {
      publishTakeoffSequence(setup_pub, control_pub, setup_topic, control_topic, frame_id);
    } else if (selection == 6) {
      publishControl(control_pub, control_topic, frame_id, sunray_msgs::UAVControlCMD::Hover, "Hover");
    } else if (selection == 7) {
      publishStop(stop_pub, stop_topic);
      ros::Duration(0.2).sleep();
      publishControl(control_pub, control_topic, frame_id, sunray_msgs::UAVControlCMD::Land, "Land");
    } else {
      printWarnLine("未知功能编号：" + std::to_string(selection));
    }

    ros::spinOnce();
  }

  (void)state_sub;
  (void)uav_state_sub;
  return 0;
}
