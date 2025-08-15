#include "mavlink_control.h"
Generic_Port *uart_port;
Autopilot_Interface *autopilot_interface_ptr;

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------
void commands(Autopilot_Interface &api, bool autotakeoff){
	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands

	if(autotakeoff){
		// arm autopilot
		api.arm_disarm(true);
		usleep(100); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command

	// Example 1 - Fly up by to 2m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z - 2.0 , // [m]
				  sp         );

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i=0; i < 8; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}


	// Example 2 - Set Velocity
	set_velocity( -1.0       , // [m/s]
				  -1.0       , // [m/s]
				   0.0       , // [m/s]
				   sp        );

	// Example 2.1 - Append Yaw Command
	set_yaw( ip.yaw + 90.0/180.0*M_PI, // [rad]
			 sp     );

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	for (int i=0; i < 4; i++){
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}

	if(autotakeoff){
		// Example 3 - Land using fixed velocity
		set_velocity(  0.0       , // [m/s]
					   0.0       , // [m/s]
					   1.0       , // [m/s]
					   sp        );

		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

		// SEND THE COMMAND
		api.update_setpoint(sp);
		// NOW pixhawk will try to move

		// Wait for 8 seconds, check position
		for (int i=0; i < 8; i++)
		{
			mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
			printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
			sleep(1);
		}

		printf("\n");

		// disarm autopilot
		api.arm_disarm(false);
		usleep(100); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands

	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );

	printf("\n");

	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------
	return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig ){
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		if(autopilot_interface_ptr)
			autopilot_interface_ptr->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		if(uart_port)
			uart_port->stop();
	}
	catch (int error){}
	// end program here
	exit(0);
}

void mavlink_init(const char *uart_name_ , int baudrate_){
	signal(SIGINT,quit_handler); // register quit handler for Ctrl-C
	uart_port = new Serial_Port(uart_name_, baudrate_);
	autopilot_interface_ptr = new Autopilot_Interface(uart_port);
    uart_port->start();
}

void mavlink_control(){
	autopilot_interface_ptr->start();
	commands(*autopilot_interface_ptr, false);
}

void mavlink_deinit(){
	autopilot_interface_ptr->stop();
	uart_port->stop();
	std::cout << "Mavlink control uninitialized." << std::endl;
}

void mavlink_save_odometry(const mavlink_odometry_t &odom){
	autopilot_interface_ptr->save_odometry(odom);
}

void mavlink_send_odometry(const mavlink_odometry_t &odom){
	autopilot_interface_ptr->send_odometry(odom);
}

void mavlink_send_odometry_thread(){
	autopilot_interface_ptr->start_odometry_thread();
}


