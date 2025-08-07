"""
Python implementation of A8 Mini SDK by SIYI
@描述:定义云台id以及各种消息类型

"""
from os import stat
from crc16_python import crc16_str_swap
import logging
from utils import toHex

class FirmwareMsg:
    seq=0
    code_board_ver=''
    gimbal_firmware_ver=''
    zoom_firmware_ver=''

class HardwareIDMsg:
    # x6B: ZR10
    # x73: A8 mini
    # x75: A2 mini
    # x78: ZR30
    # x83: ZT6
    # x7A: ZT30
    CAM_DICT ={'6B': 'ZR10', '73': 'A8 mini', '75': 'A2 mini', '78': 'ZR30', '83': 'ZT6', '7A': 'ZT30'}
    seq=0
    id=''
    cam_type_str=''

class SetUTCTimeMsg:
    seq = 0
    success = False

class SystemTimeMsg:
    seq = 0
    time_unix_usec = 0  # UNIX时间戳(微秒)
    time_boot_ms = 0    # 系统启动时间(毫秒)

class AutoFocusMsg:
    seq=0
    success=False

class ManualZoomMsg:
    seq=0
    level=-1

class ManualFocusMsg:
    seq=0
    success=False

class GimbalSpeedMsg:
    seq=0
    success=False

class CenterMsg:
    seq=0
    success=False

class DownMsg:
    seq=0
    success=False

class RecordingMsg:
    seq=0
    state=-1
    OFF=0
    ON=1
    TF_EMPTY=2
    TD_DATA_LOSS=3

class MountDirMsg:
    seq=0
    dir=-1
    NORMAL=0
    UPSIDE=1

class MotionModeMsg:
    seq=0
    mode=-1
    LOCK=0
    FOLLOW=1
    FPV=2

class StreamTypeMsg:
    seq = 0
    type = -1
    RECORD_STREAM = 0
    MAIN_STREAM = 1
    SUB_STREAM = 2

class VideoEncTypeMsg:
    seq=0
    type=-1
    H264_TYPE = 1
    H265_TYPE = 2

class Resolution_LMsg:
    seq=0
    resolution=-1
    RESOLUTION_L=0.0

class Resolution_HMsg:
    seq=0
    resolution=-1
    RESOLUTION_H=0.0

class VideoFrameRateMsg:
    seq=0
    rate=0
    FRAME_RATE=0.0

class FormatSDCardMsg:
    seq=0
    format_sta=-1 # 0=失败，1=成功

class GimbalRebootMsg:
    seq = 0
    success = False

    def parse(self, data):
        if len(data) >= 2:
            cam_ack = data[0]
            gimbal_ack = data[1]
            self.success = (cam_ack == 1 or gimbal_ack == 1)
        else:
            self.success = False

class FuncFeedbackInfoMsg:
    seq=0
    info_type=None
    SUCCESSFUL=0
    PHOTO_FAIL=1
    HDR_ON=2
    HDR_OFF=3
    RECROD_FAIL=4

class AttitdueMsg:
    seq=    0
    stamp=  0 # seconds
    yaw=    0.0
    pitch=  0.0
    roll=   0.0
    yaw_speed=  0.0 # deg/s
    pitch_speed=0.0
    roll_speed= 0.0

class SetGimbalAnglesMsg:
    seq = 0
    yaw = 0.0
    pitch = 0.0
    roll = 0.0

class RequestDataStreamMsg:
    # data_type uint8_t
    ATTITUDE_DATA = '01'
    LASER_DATA = '02'

    # Frequency
    FREQ = {0: '00', 2: '01', 4: '02', 5: '03', 10: '04', 20: '05', 50: '06', 100: '07'}

    seq = 0 
    data_type = 1 # uint8_t
    data_frequency = 0 # 0 means OFF (0, 2, 4, 5, 10, 20, 50, 100)

class RequestAbsoluteZoomMsg:
    seq = 0
    success = 0

class  CurrentZoomValueMsg:
    seq = 0
    int_part = 1
    float_part = 0
    level=0.0

class COMMAND:
    ACQUIRE_FW_VER = '01'
    ACQUIRE_HW_ID = '02'
    AUTO_FOCUS = '04'
    MANUAL_ZOOM = '05'
    MANUAL_FOCUS = '06'
    GIMBAL_SPEED = '07'
    CENTER = '08'
    ACQUIRE_GIMBAL_INFO = '0a'
    FUNC_FEEDBACK_INFO = '0b'
    PHOTO_VIDEO_HDR = '0c'
    ACQUIRE_GIMBAL_ATT = '0d'
    SET_GIMBAL_ATTITUDE = '0e'
    SET_DATA_STREAM = '25'
    ABSOLUTE_ZOOM = '0f'
    CURRENT_ZOOM_VALUE = '18'
    GIMBAL_PARAM = '20'
    SET_GIMBAL_PARAM = '21'
    SET_UTC_TIME = '30'
    ACQUIRE_TIME = '40'
    FORMAT_SDCARD = '48'
    GIMBAL_REBOOT = '80'


#############################################
class SIYIMESSAGE:
    """
    Structure of SIYI camera messages
    """
    def __init__(self, debug=False) -> None:
        self._debug= debug # print debug messages
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT='[%(levelname)s] %(asctime)s [SIYIMessage::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)


        self.HEADER='5566'# STX, 2 bytes
        self._ctr ='01'        

        self._seq= 0

        self._cmd_id='00' # 1 byte
        
        self._data_len = 0
        
        # String of data byes (in hex)
        self._data=''

        self._crc16='0000' # low byte (2 characters) on the left!

    
    def incrementSEQ(self, val):
        """
        Increments sequence number by one, converts them to hex, and revereses the byte order.

        Params
        --
        - val [int] Integer value , max is 65535

        Returns
        --
        seq_str: [string] String value of the sequence number in reveresed byte order
        """
        
        if not isinstance(val, int):
            self._logger.warning("Sequence value is not integer. Returning zero")
            return "0000"
        if val> 65535:
            self._logger.warning("Sequence value is greater than 65535. Resetting to zero")
            self._seq = 0
            return "0000"
        if val<0:
            self._logger.warning("Sequence value is negative. Resetting to zero")
            return "0000"

        seq = val+1
        self._seq = seq

        seq_hex = hex(seq)
        seq_hex = seq_hex[2:] # remove '0x'
        if len(seq_hex)==3:
            seq_hex = '0'+seq_hex
        elif len(seq_hex)==1:
            seq_hex = '000'+seq_hex
        elif len(seq_hex)==2:
            seq_str = '00'+seq_hex
        else:
            seq='0000'
        
        low_b = seq_hex[-2:]
        high_b = seq_hex[0:2]
        seq_str = low_b+high_b

        return seq_str

    def computeDataLen(self, data):
        """
        Computes the data lenght (number of bytes) of data, and return a string of two bytes in reveresed order

        Params
        --
        data [string] string of data bytes in hex

        Returns
        --
        [string] String of two bytes (for characters), in reversed order, represents length of data in hex
        """

        if not isinstance(data, str):
            self._logger.error("Data is not of type string")
            return "0000"
        # We expect number of chartacters to be even (each byte is represented by two cahrs e.g. '0A')
        if (len(data)%2) != 0:
            data = '0'+data # Pad 0 from the left, as sometimes it's ignored!
        L = int(len(data)/2)

        len_hex = hex(L)
        len_hex = len_hex[2:] # remove '0x'
        if len(len_hex)==3:
            len_hex = '0'+len_hex
        elif len(len_hex)==1:
            len_hex = '000'+len_hex
        elif len(len_hex)==2:
            len_hex = '00'+len_hex
        else:
            len_hex='0000'
        
        low_b = len_hex[-2:]
        high_b = len_hex[0:2]
        len_str = low_b+high_b

        return len_str

    def decodeMsg(self, msg):
        """
        Decodes messages string, and returns the DATA bytes.

        Params
        --
        msg: [str] full message stinf in hex

        Returns
        --
        - data [str] string of hexadecimal of data bytes.
        - data_len [int] Number of data bytes   
        - cmd_id [str] command ID
        - seq [int] message sequence
        """
        data = None
        
        if not isinstance(msg, str):
            self._logger.error("Input message is not a string")
            return data

        #  在这里打印完整收到的 message，方便调试
        # self._logger.info("Decoding message: %s", msg)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2
        if len(msg)<MINIMUM_DATA_LENGTH:
            self._logger.error("No data to decode")
            return data

        
        # Now we got minimum amount of data. Check if we have enough
        # Data length, bytes are reversed, according to SIYI SDK
        low_b = msg[6:8] # low byte
        high_b = msg[8:10] # high byte
        data_len = high_b+low_b
        data_len = int('0x'+data_len, base=16)
        char_len = data_len*2 # number of characters. Each byte is represented by two characters in hex, e.g. '0A'= 2 chars

        # check crc16, if msg is OK!
        msg_crc=msg[-4:] # last 4 characters
        payload=msg[:-4]
        expected_crc=crc16_str_swap(payload)
        if expected_crc!=msg_crc:
            self._logger.error("CRC16 is not valid. Got %s. Expected %s. Message might be corrupted!", msg_crc, expected_crc)
            return data
        
        # Sequence
        low_b = msg[10:12] # low byte
        high_b = msg[12:14] # high byte
        seq_hex = high_b+low_b
        seq = int('0x'+seq_hex, base=16)
        
        # CMD ID
        cmd_id = msg[14:16]
        
        # DATA
        if data_len>0:
            data = msg[16:16+char_len]
        else:
            data=''
        
        self._data = data
        self._data_len = data_len
        self._cmd_id = cmd_id

        return data, data_len, cmd_id, seq

    def encodeMsg(self, data, cmd_id):
        """
        Encodes a msg according to SDK protocol

        Returns
        --
        [str] Encoded msg. Empty string if crc16 is not successful
        """
        # self._logger.info(f"encodeMsg input: data={data}, cmd_id={cmd_id}")

        seq = self.incrementSEQ(self._seq)
        data_len = self.computeDataLen(data)
        # msg_front = self.HEADER+self._ctr+data_len+seq+cmd_id+data
        msg_front = self.HEADER+self._ctr+data_len+'0000'+cmd_id+data
        crc = crc16_str_swap(msg_front)
        if crc is not None:
            msg = msg_front+crc
            self._logger.debug("Encoded msg: %s", msg)
            return msg
        else:
            self._logger.error("Could not encode message. crc16 is None")
            return ''

    ########################################################
    #               Message definitions                    #
    ########################################################
    
    def firmwareVerMsg(self):
        """
        Returns message string of the Acqsuire Firmware Version msg
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_FW_VER
        return self.encodeMsg(data, cmd_id)
    
    def hwIdMsg(self):
        """
        Returns message string for the Acquire Hardware ID
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_HW_ID
        return self.encodeMsg(data, cmd_id)

    def gimbalInfoMsg(self):
        """
        Gimbal status information msg
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_GIMBAL_INFO
        return self.encodeMsg(data, cmd_id)
    
    def streamParamMsg(self):
        """
        gimbal stream param msg
        """
        data=""
        cmd_id = COMMAND.GIMBAL_PARAM
        return self.encodeMsg(data,cmd_id)
    
    def SetStreamParamMsg(self):
        """
        gimbal stream param msg
        """
        data=""
        cmd_id = COMMAND.SET_GIMBAL_PARAM
        return self.encodeMsg(data,cmd_id)
    
    def funcFeedbackMsg(self):
        """
        Function feedback information msg
        """
        data=""
        cmd_id = COMMAND.FUNC_FEEDBACK_INFO
        return self.encodeMsg(data, cmd_id)

    def takePhotoMsg(self):
        """
        Take photo msg
        """
        data="00"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def recordMsg(self):
        """
        Video Record msg
        """
        data="02"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def centerMsg(self):
        """
        Center gimbal msg
        """
        data="01"
        cmd_id = COMMAND.CENTER
        return self.encodeMsg(data, cmd_id)
    
    def downMsg(self):
        """
        down gimbal msg
        """
        data="02"
        cmd_id = COMMAND.CENTER
        return self.encodeMsg(data, cmd_id)

    def lockModeMsg(self):
        """
        Lock mode msg
        """
        data="03"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def followModeMsg(self):
        """
        Follow mode msg
        """
        data="04"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)
    
    def fpvModeMsg(self):
        """
        FPV mode msg
        """
        data="05"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def gimbalAttMsg(self):
        """
        Acquire Gimbal Attiude msg
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_GIMBAL_ATT
        return self.encodeMsg(data, cmd_id)

    def zoomInMsg(self):
        """
        Zoom in Msg
        """
        data=toHex(1,8)
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def zoomOutMsg(self):
        """
        Zoom out Msg
        """
        data=toHex(-1,8)
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def stopZoomMsg(self):
        """
        Stop Zoom Msg
        """
        data=toHex(0,8)
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def gimbalSpeedMsg(self, yaw_speed, pitch_speed):
        """
        Gimbal rotation Msg.
        Values -100~0~100: Negative and positive represent two directions,
        higher or lower the number is away from 0, faster the rotation speed is.
        Send 0 when released from control command and gimbal stops rotation.

        Params
        --
        - yaw_speed [int] in degrees
        - pitch_speed [int] in degrees
        """
        if yaw_speed>100:
            yaw_speed=100
        if yaw_speed<-100:
            yaw_speed=-100

        if pitch_speed>100:
            pitch_speed=100
        if pitch_speed<-100:
            pitch_speed=-100

        data1=toHex(yaw_speed, 8)
        data2=toHex(pitch_speed, 8)
        data=data1+data2
        cmd_id = COMMAND.GIMBAL_SPEED
        return self.encodeMsg(data, cmd_id)
    
    def setGimbalAttitude(self, target_yaw_deg, target_pitch_deg):
        """
        Set gimbal angles Msg.
        Values are in degrees and depend on the camera specs.
        The accuracy of the control angle is in one decimal place.
        Eg: Set yaw as 60.5 degrees, the command number should be set as 605.
        The actual angle data returned to be divided by 10 is the actual degree, accuracy in one decimal place.

        Params
        --
        - target_yaw_deg [in16t] in degrees up to 1 decimal. e.g. 60.5 should 605
        - pitch_speed [int16] in degrees up to 1 decimal
        """

        yaw_hex = toHex(target_yaw_deg, 16)
        pitch_hex = toHex(target_pitch_deg, 16)
        data = yaw_hex+pitch_hex
        cmd_id = COMMAND.SET_GIMBAL_ATTITUDE
        return self.encodeMsg(data, cmd_id)
    
    def setStreamParamMsg(self, stream_type, enc_type, resolution_w, resolution_h, bitrate, reserve=0):
        """
        设置相机编码参数（0x21）

        参数:
        - stream_type: uint8 (0=录像, 1=主码流, 2=副码流)
        - enc_type: uint8 (1=H264, 2=H265)
        - resolution_w: uint16
        - resolution_h: uint16
        - bitrate: uint16 (单位 kbps)
        - reserve: uint8 (保留，填 0)
        """
        data = (
            toHex(stream_type, 8) +
            toHex(enc_type, 8) +
            toHex(resolution_w, 16) +
            toHex(resolution_h, 16) +
            toHex(bitrate, 16) +
            toHex(reserve, 8)
        )
            
        self._logger.info(f"[参数设置] 流类型: {stream_type}, 编码: {enc_type}, "
                     f"分辨率: {resolution_w}x{resolution_h}, 位率: {bitrate}Kbps")
        self._logger.info(f"[参数设置] 原始数据: {data}")

        encode_msgs = self.encodeMsg(data, COMMAND.SET_GIMBAL_PARAM)
        self._logger.info(f"[DEBUG] setStreamParamMsg raw data: {encode_msgs}")
        return encode_msgs  # 0x21

    def gimbalRebootMsg(self, camera_reboot: int, gimbal_reset: int):
        """
        软重启云台相机（CMD_ID = 0x80）

        参数：
        - camera_reboot: 0=无操作，1=相机重启
        - gimbal_reset:  0=无操作，1=云台重启
        """
        data = toHex(camera_reboot, 8) + toHex(gimbal_reset, 8)
        return self.encodeMsg(data, COMMAND.GIMBAL_REBOOT)  # CMD_ID = 0x80

    def formatSDCardMsg(self):
        """
        构造格式化 SD 卡命令消息。
        """
        data = toHex(1, 8)
        cmd_id = COMMAND.FORMAT_SDCARD
        return self.encodeMsg(data, cmd_id)
    
    def setUTCTimeMsg(self, timestamp_us):
        """
        设置UTC时间消息
        :param timestamp_us: Unix时间戳(微秒)
        :return: 编码后的消息字符串
        """
        
        # 将时间戳转换为64位整数 (8字节)
        timestamp_hex = format(timestamp_us, '016X')  # 16个十六进制字符
        
        # 转换为小端序
        le_bytes = []
        for i in range(0, 16, 2):
            byte_pair = timestamp_hex[i:i+2]
            le_bytes.insert(0, byte_pair)  # 小端序: 低位在前
        
        le_timestamp = ''.join(le_bytes)
        
        # 构建数据
        data = le_timestamp
        
        cmd_id = COMMAND.SET_UTC_TIME
        return self.encodeMsg(data, cmd_id)
    
    def systemTimeMsg(self):
        """
        构造请求系统时间命令消息 (CMD_ID: 0x40)
        """
        data = ""
        cmd_id = COMMAND.ACQUIRE_TIME
        return self.encodeMsg(data, cmd_id)

    def dataStreamMsg(self, dtype: int, freq: int):
        """
        Request data stream at specific rate.
        Supported stream are
        Attitude and Laser. Laser only for ZT 30, but frequency is not supported yet. 
        Frequency is supported for attitude,

        Params
        --
        - dtype [uint8_t] 1: attitude, 2: laser
        - freq [uint8_t] frequencey options (0: OFF, 2, 4, 5,10, 20 ,50 ,100)
        """
        if dtype == 1:
            data_type_hex = RequestDataStreamMsg.ATTITUDE_DATA
        elif dtype == 2:
            data_type_hex = RequestDataStreamMsg.LASER_DATA
        else:
            self._logger.error(f"Data stream type {type} not supported. Must be 1 (atitude) or 2 (laser)")
            return ''
        
        f = int(freq)
        try:
            f_hex = RequestDataStreamMsg.FREQ[f]
        except Exception as e:
            self._logger.error(f"Frequency {freq} not supported {e}. Not requesting attitude stream.")
            return ''
        data = data_type_hex+f_hex
        cmd_id = COMMAND.SET_DATA_STREAM
        return self.encodeMsg(data, cmd_id)
    
    def absoluteZoomMsg(self, zoom_level: float):
        """
        Params
        --
        - zoom_level [float] the integer par
        """

        # Get the integer part
        integer_part = int(zoom_level)
        # Get the first decimal place as an integer
        decimal_part = int((zoom_level * 10) % 10)

        d1 = toHex(integer_part, 8)
        d2 = toHex(decimal_part, 8)
        data = d1+d2
        cmd_id = COMMAND.ABSOLUTE_ZOOM

        return self.encodeMsg(data, cmd_id)
    
    def requestCurrentZoomMsg(self):
        data=""
        cmd_id = COMMAND.CURRENT_ZOOM_VALUE
        return self.encodeMsg(data, cmd_id)