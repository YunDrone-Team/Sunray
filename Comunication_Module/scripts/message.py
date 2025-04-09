import struct

class Message:
    '''
    uint16_t head; 
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;
    uint32_t time_stamp;
    uint8_t uav_mode;
    '''
    def __init__(self):
        self.head = None
        self.length = None
        self.msg_id = None
        self.robot_id = None
        self.check = None
        self.time_stamp = None
        self.uav_mode = None

    def calculate_checksum_Mode_Message(self):
        checksum = 0
        checksum += self.head
        checksum += self.length
        checksum += self.msg_id
        checksum += self.robot_id
        checksum += self.time_stamp
        checksum += self.uav_mode
        return (checksum & 0xFFFF)
    
    def pack(self):
        format_string = '<HIBBHIB'
        packed_msg = struct.pack(format_string, self.head, self.length, self.msg_id, self.robot_id, self.check, self.time_stamp, self.uav_mode)
        return packed_msg

class Vehicle_Message:
    '''
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;
    uint32_t time_stamp;
    uint8_t type;
    '''
    def __init__(self):
        self.head = None
        self.length = None
        self.msg_id = None
        self.robot_id = None
        self.check = None
        self.time_stamp = None
        self.type = None

    def calculate_checksum_Vehicle_Message(self):
        checksum = 0
        checksum += self.head
        checksum += self.length
        checksum += self.msg_id
        checksum += self.robot_id
        checksum += self.time_stamp
        checksum += self.type
        return (checksum & 0xFFFF)
    
    def pack(self):
        format_string = '<HIBBHIB'
        packed_msg = struct.pack(format_string, self.head, self.length, self.msg_id, self.robot_id, self.check, self.time_stamp, self.type)
        return packed_msg

class Control_Message:
    '''
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;
    uint32_t time_stamp;
    uint8_t type;
    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pitch;
    bool yaw_rate;
    uint8_t frame;
    '''

    def __init__(self):
        self.head = None
        self.length = None
        self.msg_id = None
        self.robot_id = None
        self.check = None
        self.time_stamp = None
        self.type = None
        self.x = None
        self.y = None
        self.z = None
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = False
        self.frame = 0

    def calculate_checksum_Control_Message(self):
        checksum = 0
        checksum += self.head
        checksum += self.length
        checksum += self.msg_id
        checksum += self.robot_id
        checksum += self.time_stamp
        checksum += self.type
        # checksum += int(self.x)
        # checksum += int(self.y)
        # checksum += int(self.z)
        # checksum += int(self.yaw)
        # checksum += int(self.roll)
        # checksum += int(self.pitch)
        checksum += struct.unpack('I', struct.pack('f', self.x))[0]
        checksum += struct.unpack('I', struct.pack('f', self.y))[0]
        checksum += struct.unpack('I', struct.pack('f', self.z))[0]
        checksum += struct.unpack('I', struct.pack('f', self.yaw))[0]
        checksum += struct.unpack('I', struct.pack('f', self.roll))[0]
        checksum += struct.unpack('I', struct.pack('f', self.pitch))[0]
        checksum += self.yaw_rate
        checksum += self.frame
        return (checksum & 0xFFFF)

    def pack(self):
        format_string = '<HIBBHIB6fBB'
        packed_msg = struct.pack(format_string, self.head, self.length, self.msg_id, self.robot_id, self.check, 
                                 self.time_stamp, self.type, self.x, self.y, self.z, self.yaw, self.roll, self.pitch, 
                                 self.yaw_rate, self.frame)
        return packed_msg

class State_Message:
    def __init__(self):
        self.head = 0
        self.length = 0
        self.msg_id = 0
        self.robot_id = 0
        self.check = 0
        self.time_stamp = 0
        self.uav_id = 0
        self.connected = 0
        self.armed = 0
        self.mode = ""
        self.location_source = 0
        self.odom_valid = 0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.attitude_rate = [0.0, 0.0, 0.0]
        self.battery_state = 0
        self.battery_percentage = 0
        self.control_mode = 0

    def copy_from(self, tup):
        self.head = tup[0]
        self.length = tup[1]
        self.msg_id = tup[2]
        self.robot_id = tup[3]
        self.check = tup[4]
        self.time_stamp = tup[5]
        self.uav_id = tup[6]
        self.connected = tup[7]
        self.armed = tup[8]
        self.mode = tup[9]
        self.location_source = tup[10]
        self.odom_valid = tup[11]
        self.position = [tup[12], tup[13], tup[14]]
        self.velocity = [tup[15], tup[16], tup[17]]
        self.attitude = [tup[18], tup[19], tup[20]]
        self.attitude_rate = [tup[21], tup[22], tup[23]]
        self.battery_state = tup[24]
        self.battery_percentage = tup[25]
        self.control_mode = tup[26]
        # return self
        
def test_pack_Mode_Message(self):
    msg = Message()
    msg.head = 1
    msg.length = 2
    msg.msg_id = 3
    msg.robot_id = 4
    msg.time_stamp = 5
    msg.uav_mode = 6
    msg.check = msg.calculate_checksum_Mode_Message()
    packed_msg = msg.pack()


