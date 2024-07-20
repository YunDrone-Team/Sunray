std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ACC = 1	# Position Reference
uint8 IGNORE_ACC_VEL = 2	# Position Reference

geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw