// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_C_API_DRIVER_H
#define ORD_LIDAR_C_API_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "ordlidar_protocol.h"

  /// lidar instance
  typedef struct
  {
    void *lidar;
  } ORDLidar;

  //Create a LiDAR instance
  ORDLidar *oradar_lidar_create(uint8_t type, int model);
  //Destroy LiDAR instance
  void oradar_lidar_destroy(ORDLidar **lidar);
  //Set serial port properties
  bool oradar_set_serial_port(ORDLidar *lidar, char *port, int baudrate);
  // Check and open the LiDAR serial port, and create the LiDAR serial port read thread
  bool oradar_connect(ORDLidar *lidar);
  // Close the read thread of the LiDAR serial port, and close the serial port
  bool oradar_disconnect(ORDLidar *lidar);
  // Gets the timestamp of the latest package. unit: ms
  bool oradar_get_timestamp(ORDLidar *lidar, uint16_t *timestamp);
  // Get the motor speed of the latest package. unit: Hz
  bool oradar_get_rotation_speed(ORDLidar *lidarm, double *rotation_speed);
  // Get the firmware version number of the upper and lower groups
  bool oradar_get_firmware_version(ORDLidar *lidar, char *top_fw_version, char *bot_fw_version);
  // Get LiDAR Device SN
  bool oradar_get_device_sn(ORDLidar *lidar, char *device_sn);
  // Set motor speed
  bool oradar_set_rotation_speed(ORDLidar *lidar, uint16_t speed);
  // LiDAR enters ranging state from standby state
  bool oradar_activate(ORDLidar *lidar);
  // LiDAR enters standby state from ranging state
  bool oradar_deactive(ORDLidar *lidar);

	// Blocking access to the latest packet of point cloud data. 
	// Point cloud data contains the angle, distance and intensity information of all points
  bool oradar_get_grabonescan_blocking(ORDLidar *lidar, one_scan_data_st *data, int timeout_ms);
 	// Blocking access to the latest circle of point cloud data. 
	// Point cloud data contains the angle, distance and intensity information of all points
  bool oradar_get_grabfullscan_blocking(ORDLidar *lidar, full_scan_data_st *data, int timeout_ms);
  // Non-blocking access to the latest packet of point cloud data;
	// Point cloud data contains the angle, distance and intensity information of all points
  bool oradar_get_grabonescan(ORDLidar *lidar, one_scan_data_st *data);
  // Non-blocking access to the latest circle of point cloud data. 
	// Point cloud data contains the angle, distance and intensity information of all points
  bool oradar_get_grabfullscan(ORDLidar *lidar, full_scan_data_st *data);

#ifdef __cplusplus
}
#endif

#endif
