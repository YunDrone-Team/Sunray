# Message for ATTITUDE_TARGET

std_msgs/Header header

## 时间
float32 time_boot_ms                ## Timestamp (time since system boot).

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1	# attitude_rate.x
uint8 IGNORE_PITCH_RATE = 2	# attitude_rate.y
uint8 IGNORE_YAW_RATE = 4	# attitude_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128	# orientation field

float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q
float32[3]  attitude_rate
float32 thrust
