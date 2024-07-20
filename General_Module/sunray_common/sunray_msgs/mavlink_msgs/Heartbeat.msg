std_msgs/Header header

## MAV_TYPE
uint8 mav_type
## autopilot
uint8 autopilot
## base_mode
uint8 base_mode
## custom_mode
uint32 custom_mode

## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## 是否解锁，true为已解锁，false则不是
bool armed
## PX4飞控当前飞行模式
string mode

string MODE_PX4_MANUAL = MANUAL
string MODE_PX4_ACRO = ACRO
string MODE_PX4_ALTITUDE = ALTCTL
string MODE_PX4_POSITION = POSCTL
string MODE_PX4_OFFBOARD = OFFBOARD
string MODE_PX4_STABILIZED = STABILIZED
string MODE_PX4_RATTITUDE = RATTITUDE
string MODE_PX4_MISSION = AUTO.MISSION
string MODE_PX4_LOITER = AUTO.LOITER
string MODE_PX4_RTL = AUTO.RTL
string MODE_PX4_LAND = AUTO.LAND
string MODE_PX4_RTGS = AUTO.RTGS
string MODE_PX4_READY = AUTO.READY
string MODE_PX4_TAKEOFF = AUTO.TAKEOFF


    
