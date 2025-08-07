"""
Python implementation of SIYI SDK
@描述：对云台的初始函数以及功能的定义

"""
import socket
from siyi_message import *
from time import sleep, time
import logging
from utils import  toInt
import threading
import cameras
import struct


class SIYISDK:
    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug = debug
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT = ' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)


        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)
        
        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)        

        self._server_ip = server_ip
        self._port = port

        self._BUFF_SIZE = 1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rcv_wait_t = 5  # Receiving wait time
        self._socket.settimeout(self._rcv_wait_t)

        self.resetVars()

        # Stop threads flag
        self._stop = False  
        
        self._recv_thread = threading.Thread(target=self.recvLoop)

        # Connection thread
        self._last_fw_seq = 0  # used to check on connection liveness
        self._conn_loop_rate = 1  # seconds
        self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,))

        # Gimbal info thread @ 1Hz
        self._gimbal_info_loop_rate = 1
        self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop, args=(self._gimbal_info_loop_rate,))

        # Gimbal attitude thread @ 10Hz
        self._gimbal_att_loop_rate = 0.02
        self._g_att_thread = threading.Thread(target=self.gimbalAttLoop, args=(self._gimbal_att_loop_rate,))

    def resetVars(self):
        """
        Resets variables to their initial values.
        """
        self._connected = False
        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg = ManualZoomMsg()
        self._manualFocus_msg = ManualFocusMsg()
        self._gimbalSpeed_msg = GimbalSpeedMsg()
        self._center_msg = CenterMsg()
        self._down_msg = DownMsg()
        self._record_msg = RecordingMsg()
        self._mountDir_msg = MountDirMsg()
        self._motionMode_msg = MotionModeMsg()
        self._streamType_msg = StreamTypeMsg()
        self._videoEncType_msg = VideoEncTypeMsg()
        self._resolutionL_msg = Resolution_LMsg()
        self._resolutionH_msg = Resolution_HMsg()
        self._videoFrameRate_msg = VideoFrameRateMsg()
        self._gimbal_reboot_msg = GimbalRebootMsg()
        self._format_sd_msg = FormatSDCardMsg()
        self._set_utc_time_msg = SetUTCTimeMsg()
        self._system_time_msg = SystemTimeMsg()
        self._funcFeedback_msg = FuncFeedbackInfoMsg()
        self._att_msg = AttitdueMsg()
        self._set_gimbal_angles_msg = SetGimbalAnglesMsg()
        self._request_data_stream_msg = RequestDataStreamMsg()
        self._request_absolute_zoom_msg = RequestAbsoluteZoomMsg()
        self._current_zoom_level_msg = CurrentZoomValueMsg()
        self._last_att_seq = -1

        return True

    def connect(self, maxWaitTime=3.0, maxRetries=3):
        """
        Attempts to connect to the camera with retries if needed.
        
        Params
        --
        - maxWaitTime [float]: Maximum time to wait before giving up on connection (in seconds)
        - maxRetries [int]: Number of times to retry connecting if it fails
        """
        retries = 0
        while retries < maxRetries:
            try:
                # Initialize fresh thread instances for each connection attempt
                self._recv_thread = threading.Thread(target=self.recvLoop)
                self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,))
                self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop, args=(self._gimbal_info_loop_rate,))
                self._g_att_thread = threading.Thread(target=self.gimbalAttLoop, args=(self._gimbal_att_loop_rate,))
                
                self._logger.info(f"Attempting to connect to camera, attempt {retries + 1}")
                self._recv_thread.start()
                self._conn_thread.start()
                t0 = time()

                while True:
                    if self._connected:
                        self._logger.info(f"Successfully connected to camera on attempt {retries + 1}")
                        self._g_info_thread.start()
                        self._g_att_thread.start()

                        self.requestHardwareID()
                        sleep(0.2)
                        # self.requestDataStreamAttitude(50) # Not working 12 Sept 2024!
                        # sleep(0.5)
                        self.requestCurrentZoomLevel()
                        sleep(0.2)
                        return True

                    if (time() - t0) > maxWaitTime and not self._connected:
                        self._logger.error("Failed to connect to camera, retrying...")
                        self.disconnect()
                        retries += 1
                        break

            except Exception as e:
                self._logger.error(f"Connection attempt {retries + 1} failed: {e}")
                self.disconnect()
                retries += 1

        self._logger.error(f"Failed to connect after {maxRetries} retries")
        return False

    def disconnect(self):
        """
        Gracefully stops all threads, disconnects, and cleans up resources.
        """
        self._logger.info("Stopping all threads and disconnecting")
        self._stop = True  # Signal threads to stop

        # Close the socket to unblock any recvfrom() calls
        if self._socket:
            try:
                self._socket.close()
            except Exception as e:
                self._logger.error(f"Error closing socket: {e}")

        # Wait for threads to finish, if they're still alive
        if self._recv_thread.is_alive():
            self._recv_thread.join()
        if self._conn_thread.is_alive():
            self._conn_thread.join()
        if self._g_info_thread.is_alive():
            self._g_info_thread.join()
        if self._g_att_thread.is_alive():
            self._g_att_thread.join()

        # Reset the stop flag and other variables
        self.resetVars()
        self._stop = False

    def checkConnection(self):
        """
        Checks if there is a live connection to the camera by requesting the Firmware version.
        Runs in a thread at a defined frequency.
        """
        try:
            self.requestFirmwareVersion()
            sleep(0.1)
            if self._fw_msg.seq != self._last_fw_seq and len(self._fw_msg.gimbal_firmware_ver) > 0:
                self._connected = True
                self._last_fw_seq = self._fw_msg.seq
            else:
                self._connected = False
        except Exception as e:
            self._logger.error(f"Connection check failed: {e}")
            self.disconnect()

    def connectionLoop(self, t):
        """
        Periodically checks connection status and resets state if disconnected.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.checkConnection()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in connection loop: {e}")
                self.disconnect()
                break

    # def recvLoop(self):
    #     """
    #     Continuously receives data from the camera.
    #     """
    #     try:
    #         while not self._stop:
    #             try:
    #                 data, addr = self._socket.recvfrom(self._BUFF_SIZE)
    #                 if self._stop:
    #                     break
    #                 # Process the received data here...
    #             except socket.timeout:
    #                 # If stopping, exit without logging
    #                 if self._stop:
    #                     break
    #                 # Otherwise, continue waiting
    #                 continue
    #             except OSError:
    #                 # Socket closed, exit loop
    #                 break
    #     except Exception as e:
    #         self._logger.error(f"Error in receive loop: {e}")
    #     finally:
    #         self._logger.info("Exiting recvLoop")

    def isConnected(self):
        return self._connected

    def gimbalInfoLoop(self, t):
        """
        Periodically requests gimbal info.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.requestGimbalInfo()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in gimbal info loop: {e}")
                self.disconnect()

    def gimbalAttLoop(self, t):
        """
        Periodically requests gimbal attitude.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.requestGimbalAttitude()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in gimbal attitude loop: {e}")
                self.disconnect()

    def sendMsg(self, msg):
        """
        Sends a message to the camera

        Params
        --
        msg [str] Message to send
        """
        b = bytes.fromhex(msg)
        try:
            self._socket.sendto(b, (self._server_ip, self._port))
            return True
        except Exception as e:
            self._logger.error("Could not send bytes")
            return False

    def rcvMsg(self):
        data=None
        try:
            data,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            self._logger.warning("%s. Did not receive message within %s second(s)", e, self._rcv_wait_t)
        return data

    def recvLoop(self):
        self._logger.debug("Started data receiving thread")
        while( not self._stop):
            self.bufferCallback()
        self._logger.debug("Exiting data receiving thread")

    
    def bufferCallback(self):
        """
        Receives messages and parses its content
        """
        try:
            buff,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            self._logger.error(f"[bufferCallback] {e}")
            return

        buff_str = buff.hex()
        self._logger.debug("Buffer: %s", buff_str)

        # 这里添加这行日志输出
        self._logger.info("Received raw msg: %s", buff_str)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2

        HEADER='5566'
        # Go through the buffer
        while(len(buff_str)>=MINIMUM_DATA_LENGTH):
            if buff_str[0:4]!=HEADER:
                # Remove the 1st element and continue 
                tmp=buff_str[1:]
                buff_str=tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            # Data length, bytes are reversed, according to SIYI SDK
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len = high_b+low_b
            data_len = int('0x'+data_len, base=16)
            char_len = data_len*2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH+char_len)):
                # No useful data
                buff_str=''
                break
            
            packet = buff_str[0:MINIMUM_DATA_LENGTH+char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH+char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]

            if cmd_id==COMMAND.ACQUIRE_FW_VER:
                self.parseFirmwareMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_HW_ID:
                self.parseHardwareIDMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_INFO:
                self.parseGimbalInfoMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_ATT:
                self.parseAttitudeMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_PARAM:
                self.parseFrameRateMsg(data,seq)
            elif cmd_id==COMMAND.SET_GIMBAL_PARAM:
                self.parseSetVideoEncodeAckMsg(data,seq)
            elif cmd_id == COMMAND.GIMBAL_REBOOT:
                self.parseGimbalRebootAckMsg(data, seq)
            elif cmd_id == COMMAND.FORMAT_SDCARD:
                self.parseFormatSDCardAckMsg(data)
            elif cmd_id == COMMAND.SET_UTC_TIME:
                self.parseSetUTCTimeMsg(data,seq)
            elif cmd_id == COMMAND.ACQUIRE_TIME:
                self.parseSystemTimeMsg(data,seq)
            elif cmd_id==COMMAND.FUNC_FEEDBACK_INFO:
                self.parseFunctionFeedbackMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_SPEED:
                self.parseGimbalSpeedMsg(data, seq)
            elif cmd_id==COMMAND.AUTO_FOCUS:
                self.parseAutoFocusMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_FOCUS:
                self.parseManualFocusMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_ZOOM:
                self.parseZoomMsg(data, seq)
            elif cmd_id==COMMAND.CENTER:
                self.parseGimbalCenterMsg(data, seq)
            elif cmd_id==COMMAND.SET_GIMBAL_ATTITUDE:
                self.parseSetGimbalAnglesMsg(data, seq)
            elif cmd_id==COMMAND.SET_DATA_STREAM:
                self.parseRequestStreamMsg()
            elif cmd_id==COMMAND.CURRENT_ZOOM_VALUE:
                self.parseCurrentZoomLevelMsg(data, seq)
            else:
                self._logger.warning("CMD ID is not recognized")
        
        return
    
    ##################################################
    #               Request functions                #
    ##################################################    
    def requestFirmwareVersion(self):
        """
        Sends request for firmware version

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.firmwareVerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestHardwareID(self):
        """
        Sends request for Hardware ID

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.hwIdMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalAttitude(self):
        """
        Sends request for gimbal attitude

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalAttMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalInfo(self):
        """
        Sends request for gimbal information

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalInfoMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestGimbalParam(self):
        """
        Sends request for gimbal param

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.streamParamMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestSetGimbalParam(self, stream_type, enc_type, resolution_w, resolution_h, bitrate):
        """
        设置相机视频编码参数（通过 SDK）

        Returns
        --
        True: 成功,False: 失败
        """
        msg = self._out_msg.setStreamParamMsg(stream_type, enc_type, resolution_w, resolution_h, bitrate)
        if not self.sendMsg(msg):
            self._logger.error("[setVideoEncodeParam] 发送失败")
            return False
        self._logger.info("[setVideoEncodeParam] 设置指令已发送")
        return True

    def requestGimbalReboot(self, camera_reboot=0, gimbal_reset=0):
        """
        请求相机/云台软重启。
        默认 camera_reboot=1，gimbal_reset=1
        """
        msg = self._out_msg.gimbalRebootMsg(camera_reboot, gimbal_reset)
        if not self.sendMsg(msg):
            self._logger.error("[requestGimbalReboot] 重启命令发送失败")
            return False
        self._logger.info("[requestGimbalReboot] 已发送软重启命令")
        return True
    

    def requestFormatSDCard(self):
        """
        请求格式化 SD 卡。
        参数:
            format_sta: 1 表示发起格式化请求，0 无动作
        """
        msg = self._out_msg.formatSDCardMsg()
        if not self.sendMsg(msg):
            self._logger.error("[requestFormatSDCard] 格式化命令发送失败")
            return False
        self._logger.info("[requestFormatSDCard] 已发送格式化 SD 卡命令")
        return True
    
    def requestSetUTCTime(self, timestamp_us=None):
        """
        设置相机UTC时间
        :param timestamp_us: Unix时间戳(微秒)。如果为None,使用当前时间
        :return: 是否成功发送
        """
        if timestamp_us is None:
            # 获取当前时间戳(微秒)
            timestamp_us = int(time.time() * 1000000)
        
        msg = self._out_msg.setUTCTimeMsg(timestamp_us)
        if not self.sendMsg(msg):
            self._logger.error("设置UTC时间请求发送失败")
            return False
        
        self._logger.info(f"已发送设置UTC时间命令: {timestamp_us}μs")
        return True
    
    def requestSystemTime(self):
        """
        请求系统时间
        返回:
            bool: 是否成功发送请求
        """
        try:
            # 记录当前时间，用于后续计算响应延迟
            self._last_request_time = time()
            
            msg = self._out_msg.systemTimeMsg()
            self._logger.debug(f"发送系统时间请求: {msg}")
            
            if not self.sendMsg(msg):
                self._logger.error("请求系统时间命令发送失败")
                return False
                
            self._logger.info("已发送系统时间请求")
            return True
        except Exception as e:
            self._logger.error(f"请求系统时间时出错: {e}")
            return False

    def requestFunctionFeedback(self):
        """
        Sends request for function feedback msg

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.funcFeedbackMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestAutoFocus(self):
        """
        Sends request for auto focus

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.autoFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomIn(self):
        """
        Sends request for zoom in

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomInMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomOut(self):
        """
        Sends request for zoom out

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomOutMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomHold(self):
        """
        Sends request for stopping zoom

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopZoomMsg()
        
        return self.sendMsg(msg)
    
    def requestAbsoluteZoom(self, level: float):
        msg = self._out_msg.absoluteZoomMsg(level)
        return self.sendMsg(msg)
    
    def requestCurrentZoomLevel(self):
        msg = self._out_msg.requestCurrentZoomMsg()
        return self.sendMsg(msg)

    def requestLongFocus(self):
        """
        Sends request for manual focus, long shot

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.longFocusMsg()
        
        return self.sendMsg(msg)

    def requestCloseFocus(self):
        """
        Sends request for manual focus, close shot

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.closeFocusMsg()

        return self.sendMsg(msg)

    def requestFocusHold(self):
        """
        Sends request for manual focus, stop

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopFocusMsg()

        return self.sendMsg(msg)

    def requestCenterGimbal(self):
        """
        Sends request for gimbal centering

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.centerMsg()

        return self.sendMsg(msg)
    
    def requestDownGimbal(self):
        """
        Sends request for gimbal downing

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.downMsg()

        return self.sendMsg(msg)

    def requestGimbalSpeed(self, yaw_speed:int, pitch_speed:int):
        """
        Sends request for gimbal speed

        Params
        --
        yaw_speed [int] -100~0~100. away from zero -> fast, close to zero -> slow. Sign is for direction
        pitch_speed [int] Same as yaw_speed
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)

        return self.sendMsg(msg)

    def requestPhoto(self):
        """
        Sends request for taking photo
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.takePhotoMsg()

        return self.sendMsg(msg)

    def requestRecording(self):
        """
        Sends request for toglling video recording
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.recordMsg()

        return self.sendMsg(msg)

    def requestFPVMode(self):
        """
        Sends request for setting FPV mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.fpvModeMsg()

        return self.sendMsg(msg)

    def requestLockMode(self):
        """
        Sends request for setting Lock mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.lockModeMsg()

        return self.sendMsg(msg)

    def requestFollowMode(self):
        """
        Sends request for setting Follow mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.followModeMsg()

        return self.sendMsg(msg)
    
    def requestSetAngles(self, yaw_deg:float, pitch_deg:float):
        """
        Sends request to set gimbal angles

        Returns
        --
        [bool] True: success. False: fail
        """
        

        if self._hw_msg.cam_type_str == '':
            self._logger.error(f"Gimbal type is not yet retrieved. Check connection.")
            
        
        if self._hw_msg.cam_type_str == 'A8 mini':
            if yaw_deg > cameras.A8MINI.MAX_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds max {cameras.A8MINI.MAX_YAW_DEG}. Setting it to max")
                yaw_deg = cameras.A8MINI.MAX_YAW_DEG
            if yaw_deg < cameras.A8MINI.MIN_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds min {cameras.A8MINI.MIN_YAW_DEG}. Setting it to min")
                yaw_deg = cameras.A8MINI.MIN_YAW_DEG
            if pitch_deg > cameras.A8MINI.MAX_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds max {cameras.A8MINI.MAX_PITCH_DEG}. Setting it to max")
                pitch_deg = cameras.A8MINI.MAX_PITCH_DEG
            if pitch_deg < cameras.A8MINI.MIN_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds min {cameras.A8MINI.MIN_PITCH_DEG}. Setting it to min")
                pitch_deg = cameras.A8MINI.MIN_PITCH_DEG

        elif self._hw_msg.cam_type_str == 'ZR10':
            if yaw_deg > cameras.ZR10.MAX_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds max {cameras.ZR10.MAX_YAW_DEG}. Setting it to max")
                yaw_deg = cameras.ZR10.MAX_YAW_DEG
            if yaw_deg < cameras.ZR10.MIN_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds min {cameras.ZR10.MIN_YAW_DEG}. Setting it to min")
                yaw_deg = cameras.ZR10.MIN_YAW_DEG
            if pitch_deg > cameras.ZR10.MAX_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds max {cameras.ZR10.MAX_PITCH_DEG}. Setting it to max")
                pitch_deg = cameras.ZR10.MAX_PITCH_DEG
            if pitch_deg < cameras.ZR10.MIN_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds min {cameras.ZR10.MIN_PITCH_DEG}. Setting it to min")
                pitch_deg = cameras.A8MINI.MIN_PITCH_DEG
        else:
            self._logger.warning(f"Camera not supported. Setting angles to zero")
            return False

        msg = self._out_msg.setGimbalAttitude(int(yaw_deg*10), int(pitch_deg*10))

        return self.sendMsg(msg)
    
    def requestDataStreamAttitude(self, freq: int):
        """
        Send request to send attitude stream at specific frequency

        Params
        ---
        freq: [uint_8] frequency in Hz (0, 2, 4, 5, 10, 20, 50, 100)
        """
        msg = self._out_msg.dataStreamMsg(1, freq)
        return self.sendMsg(msg)
    
    def requestDataStreamLaser(self, freq: int):
        """
        Send request to send laser stream at specific frequency

        Params
        ---
        freq: [uint_8] frequency in Hz (0, 2, 4, 5, 10, 20, 50, 100)
        """
        msg = self._out_msg.dataStreamMsg(2, freq)
        return self.sendMsg(msg)

    ####################################################
    #                Parsing functions                 #
    ####################################################
    def parseFirmwareMsg(self, msg:str, seq:int):
        try:
            self._fw_msg.gimbal_firmware_ver= msg[8:16]
            self._fw_msg.seq=seq
            
            self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseHardwareIDMsg(self, msg:str, seq:int):
        try:
            self._hw_msg.seq=seq
            self._hw_msg.id = msg
            self._logger.debug("Hardware ID: %s", self._hw_msg.id)
            # first two characters define the camera ID
            
            # The numbers are reversed
            cam_id = msg[1]+msg[0]
            try:
                self._hw_msg.cam_type_str = self._hw_msg.CAM_DICT[cam_id]
            except Exception as e:
                self._logger.error(f"Camera not recognized. Key: {cam_id}")
                self._logger.error("Camera not recognized Error %s", e)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAttitudeMsg(self, msg:str, seq:int):
        
        try:
            self._att_msg.seq=seq
            self._att_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._att_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._att_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
            self._att_msg.yaw_speed = toInt(msg[14:16]+msg[12:14]) /10.
            self._att_msg.pitch_speed = toInt(msg[18:20]+msg[16:18]) /10.
            self._att_msg.roll_speed = toInt(msg[22:24]+msg[20:22]) /10.

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)", 
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            self._logger.debug("(yaw_speed, pitch_speed, roll_speed= (%s, %s, %s)", 
                                    self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
        
    def parseFrameRateMsg(self, msg: str, seq: int):
        try:
            if len(msg) < 18:
                self._logger.warn   ("消息长度不足，无法解析 GIMBAL_PARAM")
                return False
            # self._logger.info("Decoding message payload: %s", msg.hex())  # 如果 msg 是 bytes 类型

            self._streamType_msg.seq = seq
            self._streamType_msg.type = int(msg[0:2], 16)

            self._videoEncType_msg.seq = seq
            self._videoEncType_msg.type = int(msg[2:4], 16)

            self._resolutionL_msg.seq = seq
            self._resolutionL_msg.RESOLUTION_L = int(msg[6:8] + msg[4:6], 16)

            self._resolutionH_msg.seq = seq
            self._resolutionH_msg.RESOLUTION_H = int(msg[10:12] + msg[8:10], 16)

            self._videoBitrate = int(msg[14:16] + msg[12:14], 16)  # kbps

            self._videoFrameRate_msg.seq = seq
            self._videoFrameRate_msg.rate = int(msg[16:18], 16)
            self._videoFrameRate_msg.FRAME_RATE = float(self._videoFrameRate_msg.rate)

            self._logger.info(f"Stream Type: {self._streamType_msg.type}")
            self._logger.info(f"Encoding: {self._videoEncType_msg.type}")
            self._logger.info(f"Resolution: {self._resolutionL_msg.RESOLUTION_L} x {self._resolutionH_msg.RESOLUTION_H}")
            self._logger.info(f"Bitrate: {self._videoBitrate} kbps")
            self._logger.info(f"Framerate: {self._videoFrameRate_msg.FRAME_RATE} fps")
            self._logger.info(f"Full msg: {msg}")

            return True
        except Exception as e:
            self._logger.error("Error parsing GIMBAL_PARAM: %s", e)
            return False
        
    def parseSetVideoEncodeAckMsg(self, msg: str, seq: int):
        """
        解析设置视频编码参数的ACK消息 (0x21命令的响应)
        格式: [stream_type(1字节), sta(1字节)]
        """
        try:
            if len(msg) < 4:  # 需要至少4个字符(2字节)
                self._logger.error("ACK消息太短")
                return False
                
            stream_type = int(msg[0:2], 16)
            result = int(msg[2:4], 16)  # sta: 1=成功, 0=失败
            
            self._video_param_ack = {
                "seq": seq,
                "stream_type": stream_type,
                "result": result,
                "timestamp": time.time()
            }
            
            self._logger.info(f"[参数设置ACK] 流类型: {stream_type}, 结果: {result} (1=成功, 0=失败)")
            return True
        except Exception as e:
            self._logger.error(f"[参数设置ACK] 解析失败: {e}")
            return False

    def parseGimbalRebootAckMsg(self, msg: str, seq: int):
        try:
            camera_sta = int(msg[0:2], 16)
            gimbal_sta = int(msg[2:4], 16)

            self._logger.info(f"[parseGimbalRebootAckMsg] 相机重启状态: {camera_sta}, 云台重启状态: {gimbal_sta}")
            return True
        except Exception as e:
            self._logger.error(f"[parseGimbalRebootAckMsg] Error: {e}")
            return False
        
    def parseFormatSDCardAckMsg(self, msg):
        """
        解析格式化SD卡的ACK响应
        """
        if len(msg) < 1:
            self._logger.error("ACK消息太短")
            return

        format_sta = int.from_bytes(msg[0:1], byteorder='little')
        self._format_sd_msg.seq += 1
        self._format_sd_msg.format_sta = format_sta

        self._logger.info(f"[格式化SD卡 ACK] 状态: {format_sta} ({'成功' if format_sta==1 else '失败'})")

    def parseSetUTCTimeMsg(self, msg: str, seq: int):
        """
        解析设置UTC时间的响应
        :param msg: 数据部分字符串 (十六进制)
        :param seq: 序列号
        :return: 是否成功解析
        """
        try:
            # ACK是一个字节 (2个十六进制字符)
            if len(msg) < 2:
                self._logger.error("设置UTC时间响应太短")
                return False
                
            ack = int(msg[0:2], 16)
            self._set_utc_time_msg.seq = seq
            self._set_utc_time_msg.success = (ack == 1)
            
            self._logger.info(f"设置UTC时间响应: {'成功' if ack == 1 else '失败'}")
            return True
        except Exception as e:
            self._logger.error(f"解析UTC时间响应失败: {e}")
            return False
        
    def parseSystemTimeMsg(self, msg: str, seq: int):
        """
        解析系统时间响应 (0x40)
        数据格式:
          - 8字节: UNIX时间戳(微秒), 小端序
          - 4字节: 系统启动时间(毫秒), 小端序
        """
        try:
            # 记录原始数据
            self._logger.debug(f"原始系统时间数据: {msg} (长度: {len(msg)})")
            
            # 检查数据长度 (应为12字节 = 24个十六进制字符)
            if len(msg) != 24:
                self._logger.error(f"系统时间响应数据长度错误: 期望24字符, 实际{len(msg)}字符")
                return False
                
            self._system_time_msg.seq = seq
            
            # 使用struct模块直接解析小端序数据
            # 格式: < 表示小端序, Q 表示uint64_t, I 表示uint32_t
            data_bytes = bytes.fromhex(msg)
            time_unix_usec, time_boot_ms = struct.unpack('<QI', data_bytes)
            
            self._system_time_msg.time_unix_usec = time_unix_usec
            self._system_time_msg.time_boot_ms = time_boot_ms
            
            self._logger.info(f"系统时间解析成功: "
                             f"UNIX时间戳={time_unix_usec}μs, "
                             f"启动时间={time_boot_ms}ms")
            return True
        except Exception as e:
            self._logger.error(f"解析系统时间失败: {e}")
            return False

    def parseGimbalInfoMsg(self, msg:str, seq:int):
        try:
            self._record_msg.seq=seq
            self._mountDir_msg.seq=seq
            self._motionMode_msg.seq=seq
            
            self._record_msg.state = int('0x'+msg[6:8], base=16)
            self._motionMode_msg.mode = int('0x'+msg[8:10], base=16)
            self._mountDir_msg.dir = int('0x'+msg[10:12], base=16)

            self._logger.debug("Recording state %s", self._record_msg.state)
            self._logger.debug("Mounting direction %s", self._mountDir_msg.dir)
            self._logger.debug("Gimbal motion mode %s", self._motionMode_msg.mode)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAutoFocusMsg(self, msg:str, seq:int):
        
        try:
            self._autoFocus_msg.seq=seq
            self._autoFocus_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Auto focus success: %s", self._autoFocus_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseZoomMsg(self, msg:str, seq:int):
        
        try:
            self._manualZoom_msg.seq=seq
            self._manualZoom_msg.level = int('0x'+msg[2:4]+msg[0:2], base=16) /10.

            
            self._logger.debug("Zoom level %s", self._manualZoom_msg.level)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseManualFocusMsg(self, msg:str, seq:int):
        
        try:
            self._manualFocus_msg.seq=seq
            self._manualFocus_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Manual  focus success: %s", self._manualFocus_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg:str, seq:int):
        
        try:
            self._gimbalSpeed_msg.seq=seq
            self._gimbalSpeed_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalCenterMsg(self, msg:str, seq:int):
        
        try:
            self._center_msg.seq=seq
            self._center_msg.success = bool(int('0x'+msg, base=16))

            command_type = "Center" if self._out_msg.last_data == "01" else "Down"
            # self._logger.debug("Gimbal center success: %s", self._center_msg.success)
            # self._logger.debug("Gimbal down success: %s", self._down_msg.success)
            self._logger.debug(f"Gimbal {command_type} success: {self._center_msg.success}")

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseFunctionFeedbackMsg(self, msg:str, seq:int):
        
        try:
            self._funcFeedback_msg.seq=seq
            self._funcFeedback_msg.info_type = int('0x'+msg, base=16)

            
            self._logger.debug("Function Feedback Code: %s", self._funcFeedback_msg.info_type)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
        
    def parseSetGimbalAnglesMsg(self, msg:str, seq:int):
        
        try:
            self._set_gimbal_angles_msg.seq=seq

            # No need to parse the feedback angle as it is done by the parseAttitudeMsg() in a loop

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
        
    def parseRequestStreamMsg(self, msg:str, seq:int):
        
        try:
            self._request_data_stream_msg.seq=seq

            self._request_data_stream_msg.data_type = int('0x'+msg, base=16)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
        
    def parseCurrentZoomLevelMsg(self, msg: str, seq: int):
        try:
            self._current_zoom_level_msg.seq = seq
            int_part = int('0x'+msg[0:2], base=16)
            float_part = int('0x'+msg[2:4], base=16)
            self._current_zoom_level_msg.level = int_part + (float_part/10)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False


    ##################################################
    #                   Get functions                #
    ##################################################
    def getAttitude(self):
        return(self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)

    def getAttitudeSpeed(self):
        return(self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)

    def getFirmwareVersion(self):
        return(self._fw_msg.gimbal_firmware_ver)

    def getHardwareID(self):
        return(self._hw_msg.id)
    
    def getCameraTypeString(self):
        return(self._hw_msg.cam_type_str)

    def getRecordingState(self):
        return(self._record_msg.state)

    def getMotionMode(self):
        return(self._motionMode_msg.mode)

    def getMountingDirection(self):
        return(self._mountDir_msg.dir)
    
    def getStreamType(self):
        return(self._streamType_msg.type)
    
    def getVideoEncType(self):
        return(self._videoEncType_msg.type)
    
    def getResolutionLType(self):
        return(self._resolutionL_msg.RESOLUTION_L)
    
    def getResolutionHType(self):
        return(self._resolutionH_msg.RESOLUTION_H)
    
    def getVideoFrameRateType(self):
        return(self._videoFrameRate_msg.FRAME_RATE)
    
    def getFormatSDCardFeedback(self):
        return(self._format_sd_msg.format_sta)
    
    def getSystemTime(self):
        return {
            "unix_time_us": self._system_time_msg.time_unix_usec,
            "boot_time_ms": self._system_time_msg.time_boot_ms
        }
    
    def getGimbalRebootFeedback(self):
        return(self._gimbal_reboot_msg.success)

    def getFunctionFeedback(self):
        return(self._funcFeedback_msg.info_type)

    def getZoomLevel(self):
        return(self._manualZoom_msg.level)
    
    def getCurrentZoomLevel(self):
        return(self._current_zoom_level_msg.level)
    
    def getCenteringFeedback(self):
        return(self._center_msg.success)
    
    def getDowningFeedback(self):
        return(self._down_msg.success)
    
    def getDataStreamFeedback(self):
        return(self._request_data_stream_msg.data_type)

    #################################################
    #                 Set functions                 #
    #################################################
    def setGimbalRotation(self, yaw, pitch, err_thresh=1.0, kp=4):
        """
        Sets gimbal attitude angles yaw and pitch in degrees

        Params
        --
        yaw: [float] desired yaw in degrees
        pitch: [float] desired pitch in degrees
        err_thresh: [float] acceptable error threshold, in degrees, to stop correction
        kp [float] proportional gain
        """
        if (pitch >25 or pitch <-90):
            self._logger.error("desired pitch is outside controllable range -90~25")
            return

        if (yaw >45 or yaw <-45):
            self._logger.error("Desired yaw is outside controllable range -45~45")
            return

        th = err_thresh
        gain = kp
        while(True):
            self.requestGimbalAttitude()
            if self._att_msg.seq==self._last_att_seq:
                self._logger.info("Did not get new attitude msg")
                self.requestGimbalSpeed(0,0)
                continue

            self._last_att_seq = self._att_msg.seq

            yaw_err = -yaw + self._att_msg.yaw # NOTE for some reason it's reversed!!
            pitch_err = pitch - self._att_msg.pitch

            self._logger.debug("yaw_err= %s", yaw_err)
            self._logger.debug("pitch_err= %s", pitch_err)

            if (abs(yaw_err) <= th and abs(pitch_err)<=th):
                self.requestGimbalSpeed(0, 0)
                self._logger.info("Goal rotation is reached")
                break

            y_speed_sp = max(min(100, int(gain*yaw_err)), -100)
            p_speed_sp = max(min(100, int(gain*pitch_err)), -100)
            self._logger.debug("yaw speed setpoint= %s", y_speed_sp)
            self._logger.debug("pitch speed setpoint= %s", p_speed_sp)
            self.requestGimbalSpeed(y_speed_sp, p_speed_sp)

            sleep(0.1) # command frequency

def test():
    cam=SIYISDK(debug=False)

    if not cam.connect():
        exit(1)

    print("Firmware version: ", cam.getFirmwareVersion())
    
    cam.requestGimbalSpeed(10,0)
    sleep(3)
    cam.requestGimbalSpeed(0,0)
    print("Attitude: ", cam.getAttitude())
    cam.requestCenterGimbal()

    val=cam.getRecordingState()
    print("Recording state: ",val)
    cam.requestRecording()
    sleep(0.1)
    val=cam.getRecordingState()
    print("Recording state: ",val)
    cam.requestRecording()
    sleep(0.1)
    val=cam.getRecordingState()
    print("Recording state: ",val)

    print("Taking photo...")
    cam.requestPhoto()
    sleep(1)
    print("Feedback: ", cam.getFunctionFeedback())

    cam.disconnect()

if __name__=="__main__":
    test()