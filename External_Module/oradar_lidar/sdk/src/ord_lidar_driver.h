// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_DRIVER_H
#define ORD_LIDAR_DRIVER_H

#include <thread>

#include <stdlib.h>
#include <atomic>
#include <map>
#include <vector>
#include <core/common/ChannelDevice.h>
#include <core/base/locker.h>
#include <core/base/thread.h>
#include <core/base/timer.h>
#include "ordlidar_protocol.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The Oradar LIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace std;
//using namespace ordlidar;
namespace ordlidar
{
	using namespace core;
	using namespace base;
	using namespace core::common;
	class OrdlidarDriver
	{
	public:
		/**
		* @brief create object
		* @param[in] type          LiDAR communication type
		* @param[in] model         LiDAR model
		*/
		OrdlidarDriver(uint8_t type, int model);

		/**
		* @brief destroy object
		*/
		~OrdlidarDriver();

		/**
		* @brief Set serial port properties
		* @param[in] port_name          serial port name
		* @param[in] baudrate           serial port baudrate
		* @return true if properties is set successfully, otherwise false.
		*/
		bool SetSerialPort(const std::string &port_name, const uint32_t &baudrate);

		/**
		* @brief Check and open the LiDAR serial port, and create the LiDAR serial port read thread
		* @return true if connect successfully, otherwise false.
		*/
		bool Connect();

		/**
		* @brief Close the read thread of the LiDAR serial port, and close the serial port
		* @return None
		*/
		void Disconnect();
		
		/**
		* @brief Judge whether the LiDAR serial port is open
		* @return true if connected, otherwise false.
		*/
		bool isConnected() const;

		/**
		* @brief LiDAR enters ranging state from standby state
		* @return true if set successfully, otherwise false.
		*/
		bool Activate();

		/**
		* @brief LiDAR enters standby state from ranging state
		* @return true if set successfully, otherwise false.
		*/
		bool Deactive();

		/**
		* @brief Non-blocking access to the latest packet of point cloud data,
		*		Point cloud data contains the angle, distance and intensity information of all points
		* @param[out] scan_data           LiDAR Scan Sata
		* @return true if get scan data successfully, otherwise false.
		*/
		bool GrabOneScan(one_scan_data_st &scan_data);

		/**
		* @brief Blocking access to the latest packet of point cloud data. 
		*		Point cloud data contains the angle, distance and intensity information of all points
		* @param[out] scan_data           LiDAR Scan Sata
		* @param[in] timeout_ms          timeout ,uint:ms
		* @return true if get scan data successfully, otherwise false.
		*/
		bool GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms);

		/**
		* @brief Non-blocking access to the latest circle of point cloud data. 
		*		Point cloud data contains the angle, distance and intensity information of all points
		* @param[out] scan_data           LiDAR Scan Sata
		* @return true if get scan data successfully, otherwise false.
		*/
		bool GrabFullScan(full_scan_data_st &scan_data);

		/**
		* @brief Blocking access to the latest circle of point cloud data. 
		*		Point cloud data contains the angle, distance and intensity information of all points
		* @param[out] scan_data           LiDAR Scan Sata
		* @param[in] timeout_ms          timeout ,uint:ms
		* @return true if get scan data successfully, otherwise false.
		*/
		bool GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms);

		/**
		* @brief Gets the timestamp of the latest package. unit: ms
		* @return LiDAR current timestamp
		*/
		uint16_t GetTimestamp() const;

		/**
		* @brief  Get the motor speed of the latest package. unit: Hz
		* @return motor speed
		*/
		double GetRotationSpeed() const;

		/**
		* @brief Set motor speed
		* @param[in] speed       motor speed,unit:Hz   
		* @return true if set successfully, otherwise false.
		*/
		bool SetRotationSpeed(int speed);

		/**
		* @brief Get the firmware version number of the top and bottom boards
		* @param[out] top_fw_version       top board firmware version   
		* @param[out] bot_fw_version       bottom board firmware version   
		* @return true if get successfully, otherwise false.
		*/
		bool GetFirmwareVersion(std::string &top_fw_version, std::string &bot_fw_version);

		/**
		* @brief Get LiDAR Device SN
		* @param[out] device_sn       LiDAR Device SN  
		* @return true if get successfully, otherwise false.
		*/
		bool GetDeviceSN(std::string &device_sn);

	private:
		static void mRxThreadProc(void *arg);
		int read(unsigned char *data, int length);
		int write(unsigned char *data, int length);
		bool uart_data_handle(unsigned char *data, int len);
		bool uart_data_find_init_info(unsigned char *data, int len);
		bool IsFullScanReady(void) { return full_scan_ready_; }
		void ResetFullScanReady(void) { full_scan_ready_ = false; }
		bool IsOneScanReady(void) { return one_scan_ready_; }
		void ResetOneScanReady(void) { one_scan_ready_ = false; }
		//int point_data_parse_frame_ms200(point_data_t *data, unsigned char *buf, unsigned short buf_len, float start_angle, float end_angle);
		int point_data_parse_frame_ms200(point_data_t *data, OradarLidarFrame *pkg);

	private:
		// serial port
		ChannelDevice *serial_;
		std::string port_name_;
		std::string top_fw_version_;
		std::string bot_fw_version_;
		std::string device_sn_;
		uint32_t baudrate_;
		//tranformer type
  		uint8_t tranformer_type_;
  		int model_;
		bool is_connected_;
		bool full_scan_ready_;
		bool one_scan_ready_;
		int valid_data_;
		uint8_t init_info_flag_;
		std::vector<uint8_t> bin_buf_;
		std::vector<uint8_t> cmd_buf_;
		parsed_data_st parsed_data_;
		full_scan_data_st full_scan_data_;
		full_scan_data_st temp_data;
		one_scan_data_st one_scan_data_;
		std::thread *rx_thread_;
		std::atomic<bool> rx_thread_exit_flag_;

		Event full_data_event_;
		Event one_data_event_;
		//Locker lock_;
	};

}

#endif
