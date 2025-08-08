#!/usr/bin/env python3
# @file gimbal_control_node.py
# @描述: 云台统一控制节点，集中管理 SDK 实例，处理全部功能话题

import sys
import os
import threading
import rospy
import ffmpeg
import time
import signal  
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, UInt8
from std_msgs.msg import Float32, String
from sensor_msgs.msg import CompressedImage
from sunray_gimbal.srv import GimbalStatus, GimbalStatusResponse
from sunray_gimbal.srv import GimbalReboot, GimbalRebootResponse
from std_srvs.srv import Trigger, TriggerResponse
from sunray_gimbal.msg import GimbalParams

import cv2
import numpy as np
from time import sleep

# 导入 SDK
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)
from utils.siyi_sdk import SIYISDK
#from stream import SIYIRTSP

#rtsp获取视频流配置器
class RTSPStreamer:
    def __init__(self, rtsp_url, width=1280, height=720):
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.process = (
            ffmpeg
            .input(self.rtsp_url, rtsp_transport='tcp')
            .output('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{width}x{height}')
            .run_async(pipe_stdout=True, pipe_stderr=True)
        )

    def get_frame(self):
        frame_size = self.width * self.height * 3
        raw_frame = self.process.stdout.read(frame_size)
        if len(raw_frame) != frame_size:
            return None
        frame = np.frombuffer(raw_frame, np.uint8).reshape((self.height, self.width, 3))
        return frame

class GimbalControlNode:
    def __init__(self):
        rospy.init_node("gimbal_control_node", anonymous=False)

        # 添加关闭节点时的软重启标志
        self.shutdown_reboot_requested = False

        # 读取参数
        ip = rospy.get_param("~gimbal_ip", "192.168.144.25")
        port = rospy.get_param("~gimbal_port", 37260)
        rtsp_url = rospy.get_param("~rtsp_url", f"rtsp://{ip}:8554/main.264")

        # 连接云台
        self.cam = SIYISDK(server_ip=ip, port=port)
        if not self.cam.connect():
            rospy.logerr("云台连接失败")
            sys.exit(1)

        # 初始化 RTSP 拉流器（使用 ffmpeg）
        self.width = rospy.get_param("~frame_width", 1280)
        self.height = rospy.get_param("~frame_height", 720)
        self.fps = rospy.get_param("~gimbal_fps", 30)
        self.rtsp_url = rospy.get_param("~rtsp_url", f"rtsp://{ip}:8554/main.264")

        # 启动视频发布线程
        self.streamer = RTSPStreamer(self.rtsp_url, self.width, self.height)
        self.stream_thread = threading.Thread(target=self.video_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()

        #订阅云台一键复位话题
        rospy.Subscriber("/sunray/gimbal_reset", Bool, self.reset_callback)
        #订阅设置目标角度话题
        rospy.Subscriber("/sunray/gimbal_set_angles", Vector3, self.angle_callback)
        #订阅设置目标速度话题
        rospy.Subscriber("/sunray/gimbal_set_speed", Vector3, self.speed_callback)
        #订阅一键回中话题<exec_depend>message_runtime</exec_depend>
        rospy.Subscriber("/sunray/gimbal_center_cmd", Bool, self.center_callback)
        #订阅一键朝下话题<exec_depend>message_runtime</exec_depend>
        rospy.Subscriber("/sunray/gimbal_down_cmd", Bool, self.down_callback)
        #订阅模式切换话题
        rospy.Subscriber("/sunray/gimbal_mode_switch", UInt8, self.mode_callback)
        #订阅绝对变倍话题
        rospy.Subscriber("/sunray/gimbal_absolute_zoom", Float32, self.absolute_zoom_callback)
        #订阅手动变倍话题
        rospy.Subscriber("/sunray/gimbal_manual_zoom", String, self.manual_zoom_callback)
        #订阅拍照和录视频话题
        rospy.Subscriber("/sunray/gimbal_take_photo", Bool, self.take_photo_callback)
        rospy.Subscriber("/sunray/gimbal_toggle_record", Bool, self.toggle_record_callback)
        #订阅设置相机参数话题
        rospy.Subscriber("/sunray/gimbal_set_param",GimbalParams,self.video_param_callback)

        #云台状态服务
        rospy.Service('/sunray/gimbal_get_status', GimbalStatus, self.handle_status_request)
        #相机云台软重启服务
        rospy.Service('/sunray/gimbal_reboot', GimbalReboot, self.handle_gimbal_reboot)
        #格式化SD卡服务
        rospy.Service("/sunray/gimbal_format_sd_card", Trigger, self.handle_format_sd_card)
        #设置UTC服务
        rospy.Service('/set_gimbal_utc_time', Trigger, self.handle_set_utc_time)

        # 创建视频话题发布器
        self.video_pub = rospy.Publisher("/camera/video_stream", CompressedImage, queue_size=10)
        # 姿态发布器
        self.attitude_pub = rospy.Publisher("/sunray/gimbal_attitude", Vector3, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_attitude)
        #云台信息发布器
        self.device_info_pub = rospy.Publisher("/sunray/gimbal_info", String, queue_size=1)
        #云台参数发布
        self.gimbal_param_pub = rospy.Publisher("/sunray/gimbal_param",GimbalParams,queue_size=1)


        sleep(1.0)
        self._publish_device_info()
        self._publish_video_params()
        rospy.loginfo("云台主控节点已启动，监听所有控制话题...")
        rospy.loginfo(f"开始拉取 RTSP 流：{rtsp_url}")

        # 注册节点关闭时的回调函数
        rospy.on_shutdown(self.shutdown_hook)

        #注册信号处理程序
        signal.signal(signal.SIGINT, self.signal_handler)  # Ctrl+C
        signal.signal(signal.SIGHUP, self.signal_handler)  # 终端关闭


    def signal_handler(self, signum, frame):
        """处理所有终止信号"""
        rospy.logwarn(f"收到终止信号 {signum}，正在关闭节点...")
        # 手动触发关闭流程
        self.shutdown_hook()
        # 退出程序
        rospy.signal_shutdown("收到终止信号")

    # 添加节点关闭时的回调函数
    def shutdown_hook(self):
        """节点关闭时触发的回调函数"""
        if not self.shutdown_reboot_requested:
            rospy.loginfo("节点关闭，正在发送相机软重启命令...")
            try:
                # 发送软重启命令（只重启相机部分）
                self.cam.requestGimbalReboot(camera_reboot=1, gimbal_reset=1)
                rospy.loginfo("相机软重启命令已发送")
                self.shutdown_reboot_requested = True
            except Exception as e:
                rospy.logerr(f"发送软重启命令失败: {str(e)}")
        else:
            rospy.loginfo("软重启命令已发送过，跳过重复操作")

    #重新启动RTSP拉流器
    def restart_rtsp_stream(self):
        rospy.loginfo("准备重新启动 RTSP 拉流器...")
        try:
            # 停止原来的 ffmpeg 子进程
            if self.streamer and self.streamer.process:
                self.streamer.process.terminate()
                self.streamer.process.wait()
        except Exception as e:
            rospy.logwarn(f"终止 RTSP 流失败: {e}")

        # 重新创建拉流器并启动线程
        self.streamer = RTSPStreamer(self.rtsp_url, self.width, self.height)
        self.stream_thread = threading.Thread(target=self.video_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        rospy.loginfo("成功启动 RTSP 拉流器...")

    def auto_set_utc_time(self, event):
        """自动设置相机UTC时间"""
        rospy.loginfo("正在设置相机UTC时间...")
        response = self.handle_set_utc_time(None)
        if response.success:
            rospy.loginfo("UTC时间设置成功")
        else:
            rospy.logwarn(f"UTC时间设置失败: {response.message}")

    #发送原始相机参数数据包
    def send_raw_set_params(self, stream_type, enc_type, width, height, bitrate):
        """
        直接发送原始UDP数据包设置相机参数
        格式: 5566 [CTRL] [LEN] [SEQ] 21 [DATA] [CRC]
        """
        # 1. 构建消息头部
        header = "5566"  # STX
        ctrl = "01"     # CTRL
        
        # 2. 构建数据部分
        data = (
            format(stream_type, '02X') +   # stream_type (1字节)
            format(enc_type, '02X') +      # enc_type (1字节)
            format(width, '04X')[2:4] + format(width, '04X')[0:2] +  # width (小端序2字节)
            format(height, '04X')[2:4] + format(height, '04X')[0:2] +  # height (小端序2字节)
            format(bitrate, '04X')[2:4] + format(bitrate, '04X')[0:2] +  # bitrate (小端序2字节)
            "00"  # reserve (1字节)
        )
        # 3. 计算数据长度 (9字节)
        data_len = len(data) // 2  # 字节数
        data_len_hex = format(data_len, '04X')
        data_len_le = data_len_hex[2:4] + data_len_hex[0:2]  # 小端序
        
        # 4. 序列号 (固定为0000)
        seq = "0000"
        
        # 5. 命令ID
        cmd_id = "21"
        
        # 6. 构建完整载荷 (不含CRC)
        payload = header + ctrl + data_len_le + seq + cmd_id + data
        
        # 7. 计算CRC16
        from utils.crc16_python import crc16_str_swap
        crc = crc16_str_swap(payload)
        
        # 8. 完整消息
        full_msg = payload + crc
        
        rospy.loginfo(f"发送原始设置命令: {full_msg}")
        
        # 9. 转换为字节并发送
        try:
            b = bytes.fromhex(full_msg)
            self.cam._socket.sendto(b, (self.cam._server_ip, self.cam._port))
            rospy.loginfo("原始设置命令已发送")
            return True
        except Exception as e:
            rospy.logerr(f"发送失败: {e}")
            return False

    #云台复位回调
    def reset_callback(self, msg):
        if not msg.data:
            rospy.loginfo("收到 False,不执行复位")
            return

        rospy.loginfo("正在执行云台复位 + 缩放重置")

        # 1. 回中
        self.cam.requestCenterGimbal()
        sleep(2)  # 等待回中完成
        self.cam.requestFunctionFeedback()
        sleep(0.1)
        center_feedback = self.cam.getCenteringFeedback()
        rospy.loginfo(f"云台回中反馈: {center_feedback}")

        # 2. 绝对变倍复位def _publish_device_info(self):
        self.cam.requestAbsoluteZoom(1.0)
        sleep(1)
        current_zoom = self.cam.getCurrentZoomLevel()
        rospy.loginfo(f"当前变倍倍数: {current_zoom}")

    #角度回调
    def angle_callback(self, msg):
        yaw, pitch = msg.x, msg.y
        if not (-135 <= yaw <= 135 and -90 <= pitch <= 25):
            rospy.logwarn(f"角度超限:yaw={yaw}, pitch={pitch}")
            return
        self.cam.requestSetAngles(yaw, pitch)
        rospy.loginfo(f"设置角度:yaw={yaw}, pitch={pitch}")

    #速度回调
    def speed_callback(self, msg):
        yaw_speed, pitch_speed = int(msg.x), int(msg.y)
        if not (-100 <= yaw_speed <= 100 and -100 <= pitch_speed <= 100):
            rospy.logwarn(f"速度超限:yaw_speed={yaw_speed}, pitch_speed={pitch_speed}")
            return
        self.cam.requestGimbalSpeed(yaw_speed, pitch_speed)
        rospy.loginfo(f"设置速度:yaw_speed={yaw_speed}, pitch_speed={pitch_speed}")

    #回中回调
    def center_callback(self, msg):
        if msg.data:
            self.cam.requestCenterGimbal()
            sleep(1)
            self.cam.requestFunctionFeedback()
            sleep(0.1)
            status = self.cam.getCenteringFeedback()
            rospy.loginfo(f"回中反馈: {status}")
        else:
            rospy.logwarn("收到 False,未执行回中")

    #朝下回调
    def down_callback(self,msg:Bool):

        if msg.data:
            angle_pub = rospy.Publisher("/sunray/gimbal_set_angles", Vector3, queue_size=1)
            rospy.sleep(0.1)  # 等待连接
            down_msg = Vector3()
            down_msg.x = 0.0   # yaw 保持水平朝前
            down_msg.y = -90.0 # pitch 朝下
            down_msg.z = 0.0
            angle_pub.publish(down_msg)
            rospy.loginfo("已执行一键朝下")
        else:
            rospy.loginfo("取消一键朝下指令")

    #模式切换回调
    def mode_callback(self, msg):
        mode = msg.data
        if mode == 0:
            self.cam.requestLockMode()
            rospy.loginfo("切换至 Lock 锁定模式")
        elif mode == 1:
            self.cam.requestFollowMode()
            rospy.loginfo("切换至 Follow 跟随模式")
        elif mode == 2:
            self.cam.requestFPVMode()
            rospy.loginfo("切换至 FPV 模式")
        else:
            rospy.logwarn(f"无效模式值：{mode}")
    def getResolutionLType(self):
        return(self._resolutionL_msg.resolution)
    
    #绝对放大倍数回调
    def absolute_zoom_callback(self, msg):
        zoom_level = msg.data
        if not (1.0 <= zoom_level <= 6.0):  # 范围根据你实际设备支持
            rospy.logwarn(f"绝对变倍范围无效: {zoom_level}")
            return
        self.cam.requestAbsoluteZoom(zoom_level)
        sleep(0.5)
        current = self.cam.getCurrentZoomLevel()
        rospy.loginfo(f"设置变倍至 {zoom_level}，当前变倍: {current}")
    def getResolutionLType(self):
        return(self._resolutionL_msg.resolution)
    
    #手动放大倍数回调
    def manual_zoom_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == "in":
            self.cam.requestZoomIn()
            rospy.loginfo("开始放大变倍")
        elif cmd == "out":
            self.cam.requestZoomOut()
            rospy.loginfo("开始缩小变倍")
        elif cmd == "hold":
            self.cam.requestZoomHold()
            rospy.loginfo("停止变倍")
        else:
            rospy.logwarn(f"无效的变倍命令: {cmd}")

    #拍照回调
    def take_photo_callback(self, msg):
        if msg.data:
            # 请求拍照
            success = self.cam.requestPhoto()
            sleep(0.3)  # 给设备一点响应时间

            # 获取录像状态，判断是否有 TF 卡
            record_state = self.cam.getRecordingState()
            if record_state == self.cam._record_msg.TF_EMPTY:
                rospy.logwarn("无 TF 卡，无法拍照！")
            elif not success:
                rospy.logwarn("拍照请求失败")
            else:
                rospy.loginfo("拍照成功")

    #录像回调
    def toggle_record_callback(self, msg):
        current_state = self.cam.getRecordingState()
        
        # 状态检查
        if current_state == self.cam._record_msg.TF_EMPTY:
            rospy.logwarn("无 TF 卡，无法操作！")
            return
            
        # 执行操作
        if msg.data and current_state == self.cam._record_msg.OFF:
            self.cam.requestRecording()  # 开启录像
        elif not msg.data and current_state == self.cam._record_msg.ON:
            self.cam.requestRecording()  # 关闭录像
        
        # 结果检查
        sleep(1)
        new_state = self.cam.getRecordingState()
        
        if msg.data and new_state == self.cam._record_msg.ON:
            rospy.loginfo("已开始录像")
        elif not msg.data and new_state == self.cam._record_msg.OFF:
            rospy.loginfo("已停止录像")
        else:
            rospy.logwarn(f"操作未生效，当前状态: {new_state}")

    #设置相机参数回调
    def video_param_callback(self,msg:GimbalParams):
        # 提取参数
        stream_type = msg.stream_type
        enc_type = msg.encoding_type
        resolution_w = msg.resolution_width
        resolution_h = msg.resolution_height
        bitrate = msg.bitrate_kbps

        # 发送设置指令
        success = self.send_raw_set_params(
            stream_type=stream_type,
            enc_type=enc_type,
            width=resolution_w,
            height=resolution_h,
            bitrate=bitrate
        )

        if not success:
            rospy.logerr("原始参数设置失败")
            return

        rospy.loginfo("设置完参数，准备软重启...")
        self.cam.requestGimbalReboot(camera_reboot=1, gimbal_reset=0)
        rospy.sleep(8)

        self.restart_rtsp_stream()

        if enc_type == 1:
            enc_type = "H264"
        elif enc_type ==2:
            enc_type = "H265"

        if success:
            rospy.loginfo(f"[设置成功] 编码方式={enc_type} 分辨率={resolution_w}x{resolution_h}")
        else:
            rospy.logwarn(f"[设置失败] 视频参数发送失败")

        self._publish_video_params()
        rospy.loginfo("参数更新完成")

    #云台状态处理服务
    def handle_status_request(self, req):
        self.cam.requestGimbalInfo()
        sleep(0.5)

        # 录像状态
        rec_state = self.cam.getRecordingState()
        if rec_state == 0:
            recording = "未录像"
        elif rec_state == 1:
            recording = "录制中"
        elif rec_state == 2:
            recording = "无TF卡"
        else:
            recording = "未知"

        # 模式
        mode = self.cam.getMotionMode()
        if mode == 0:
            mode_str = "锁定模式"
        elif mode == 1:
            mode_str = "跟随模式"
        elif mode == 2:
            mode_str = "FPV模式"
        else:
            mode_str = "未知"

        # 安装方向
        direction = self.cam.getMountingDirection()
        if direction == 0:
            dir_str = "预留"
        elif direction == 1:
            dir_str = "正常"
        elif direction == 2:
            dir_str = "倒立"
        else:
            dir_str = "未知"

        rospy.loginfo(f"\n配置状态:\n 录像状态:{recording}\n 模式:{mode_str}\n 安装:{dir_str}\n")
        return GimbalStatusResponse(recording_state=recording,
                                    motion_mode=mode_str,
                                    mounting_direction=dir_str)
    
    #相机云台软重启服务
    def handle_gimbal_reboot(self, req):
        rospy.loginfo(f"收到重启请求: camera={req.camera_reboot}, gimbal={req.gimbal_reset}")
        
        # 发送请求
        success = self.cam.requestGimbalReboot(
            camera_reboot=req.camera_reboot,
            gimbal_reset=req.gimbal_reset
        )
        if not success:
            return GimbalRebootResponse(success=False, camera_reboot_sta=0, gimbal_reset_sta=0)

        rospy.sleep(5)

        # 获取 ACK 状态
        ack = self.cam.getGimbalRebootFeedback()
        
        try:
            camera_sta = ack.camera_reboot_sta
            gimbal_sta = ack.gimbal_reset_sta
        except AttributeError:
            camera_sta = 0
            gimbal_sta = 0

        rospy.loginfo(f"重启反馈: camera={camera_sta}, gimbal={gimbal_sta}")
        
        return GimbalRebootResponse(
            success=(camera_sta == 1 or gimbal_sta == 1),
            camera_reboot_sta=camera_sta,
            gimbal_reset_sta=gimbal_sta
        )
    
    #相机SD卡格式化服务
    def handle_format_sd_card(self, req):
        # 发送格式化命令
        success = self.cam.requestFormatSDCard()
        rospy.sleep(1.0)

        if not success:
            return TriggerResponse(success=False, message="格式化命令发送失败")

        # 可选：读取反馈
        ack = self.cam.getFormatSDCardFeedback()
        if ack == 1:
            rospy.loginfo("SD卡格式化成功")
            return TriggerResponse(success=True, message="SD 卡格式化成功")
        else:
            rospy.loginfo("SD 卡格式化失败或未知状态")
            return TriggerResponse(success=False, message="SD 卡格式化失败或未知状态")
        
    
    def handle_set_utc_time(self, req):
        """设置相机UTC时间服务"""
        try:
            # # 获取当前时间（秒）
            # now_sec = rospy.get_time()
            
            # # 转换为微秒（μs）
            # timestamp_us = int(now_sec * 1000000)

            # 获取当前系统UTC时间（微秒）
            timestamp_us = int(time.time() * 1000000)
            
            # 验证时间戳范围
            min_valid = 0  # 1970-01-01
            max_valid = 2000_000_000_000_000  # 2030年后的某个时间
            
            if not (min_valid <= timestamp_us <= max_valid):
                rospy.logerr(f"无效时间戳: {timestamp_us} (超出有效范围)")
                return TriggerResponse(success=False, message="时间超出有效范围")
            
            #转换为可读时间用于日志
            from datetime import datetime,timezone,timedelta
            beijing_time = datetime.fromtimestamp(timestamp_us / 1_000_000, tz=timezone(timedelta(hours=8)))
            human_time = beijing_time.strftime("%Y-%m-%d %H:%M:%S")
            rospy.loginfo(f"\nUNIX时间戳: {timestamp_us}μs \n北京时间为: {human_time}")

            # 发送设置命令
            success = self.cam.requestSetUTCTime(timestamp_us)
            if not success:
                rospy.logerr("发送设置UTC时间命令失败")
                return TriggerResponse(success=False, message="发送命令失败")
            
            # 等待ACK (最多3秒)
            timeout = 3.0
            start_time = rospy.get_time()
            ack_received = False
            ack = None
            
            while (rospy.get_time() - start_time) < timeout and not rospy.is_shutdown():
                #检查是否收到ACK响应
                if self.cam._set_utc_time_msg.success is not None:
                    ack_received = True
                    ack = self.cam._set_utc_time_msg.success
                    break
                rospy.sleep(0.1)
            
            if not ack_received:
                rospy.logerr("设置UTC时间: 等待ACK超时")    
                return TriggerResponse(success=False, message="未收到响应")
            
            
            else:
                return TriggerResponse(success=True, message="UTC时间设置成功")
        
        except Exception as e:
            rospy.logerr(f"设置UTC时间异常: {str(e)}")
            return TriggerResponse(success=False, message=str(e))
                
    #姿态发布
    def publish_attitude(self, event):
        yaw, pitch, roll = self.cam.getAttitude()
        msg = Vector3(x=yaw, y=pitch, z=roll)
        self.attitude_pub.publish(msg)

    #云台信息发布
    def _publish_device_info(self):
        self.cam.requestHardwareID()
        sleep(0.3)
        hardware_id = self.cam.getHardwareID()
        cam_type = self.cam.getCameraTypeString()

        self.cam.requestFirmwareVersion()
        sleep(0.3)
        fw1_version = self.cam.getFirmwareVersion()
        fw2_version = self.cam.getCamFirmwareVersion()

        self.cam.requestGimbalInfo()
        sleep(0.3)
        mode = self.cam.getMotionMode()
        if mode == 0:
            mode_str = "锁定模式"
        elif mode == 1:
            mode_str = "跟随模式"
        elif mode == 2:
            mode_str = "FPV模式"
        else:
            mode_str = "未知"

        info_str = f"\n\033[36m[设备信息] \n硬件ID: {hardware_id} \n相机类型: {cam_type} \n相机固件版本: {fw2_version}\n云台固件版本: {fw1_version} \n当前模式: {mode_str}\033[0m"
        rospy.logwarn("\n==== 云台信息 ====")
        rospy.loginfo(info_str)

        # 发布一次性信息
        self.device_info_pub = rospy.Publisher("/sunray/gimbal_info", String, queue_size=1, latch=True)
        self.device_info_pub.publish(info_str)

    #相机云台参数发布
    def _publish_video_params(self):
        # 请求参数
        self.cam.requestGimbalParam()
        sleep(0.5)

        # 构造消息
        msg = GimbalParams()
        msg.stream_type = self.cam.getStreamType()
        if msg.stream_type == 1:
            stream_type = "主码流"
        elif msg.stream_type == 2:
            stream_type = "副码流"
        else:
            stream_type = "未知"

        msg.encoding_type = self.cam.getVideoEncType()
        if msg.encoding_type == 1:
            encoding_type ="H264"
        elif msg.encoding_type == 2:
            encoding_type ="H265" 
        else:
            encoding_type = "未知"

        msg.resolution_width = int(self.cam.getResolutionLType())
        msg.resolution_height = int(self.cam.getResolutionHType())
        msg.frame_rate = self.cam.getVideoFrameRateType()

        # 构造文本信息用于打印
        params_str = f"\n\033[36m[视频参数] \n编码方式: {encoding_type} \n分辨率: {msg.resolution_width}x{msg.resolution_height}\n帧率: {msg.frame_rate} fps\033[0m"

        # 打印日志
        rospy.logwarn("\n==== 视频参数 ====")
        rospy.loginfo(params_str)
        
        # 发布话题
        self.gimbal_param_pub.publish(msg)

    #视频流拉取并转换
    def video_loop(self):
        rate = rospy.Rate(self.fps)
        drop_count = 0 #丢帧计数器
        drop_limit = 40 #最大丢帧次数，超过就重启流

        while not rospy.is_shutdown():
            frame = self.streamer.get_frame()
            if frame is None or frame.size == 0:
                drop_count+=1
                rospy.logwarn_throttle(5, f"[WARN] No frame received from RTSP stream. (count={drop_count})")

                if drop_count >= drop_limit:
                    rospy.logwarn(f"[ERROR] 连续 {drop_limit} 次未收到帧，尝试重启 RTSP 拉流...")
                    self.restart_rtsp_stream()
                    return  # 退出当前线程，由重启逻辑开启新线程

                rate.sleep()
                continue
            drop_count = 0
                
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            _, buffer = cv2.imencode('.jpg', frame)
            msg.data = buffer.tobytes()

            self.video_pub.publish(msg)
            rate.sleep()


    def run(self):
        try:
            rospy.spin()
        finally:
            # 确保无论如何都断开连接
            if hasattr(self, 'cam'):
                self.cam.disconnect()
                rospy.loginfo("已安全断开云台连接")

if __name__ == "__main__":
    GimbalControlNode().run()
