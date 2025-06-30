#include <cmath>
#include <time.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>
#include <sys/time.h>

using std::string;
using namespace std;
#include <common/mavlink.h>
#include "serial_port.h"
#include "autopilot_interface.h"

void mavlink_init(const char *uart_name_ , int baudrate_);
void mavlink_control();
void mavlink_save_odometry(const mavlink_odometry_t &odom);
void mavlink_send_odometry(const mavlink_odometry_t &odom);
void mavlink_send_odometry_thread();
void mavlink_deinit();
