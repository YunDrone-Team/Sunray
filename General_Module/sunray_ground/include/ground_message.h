#include <cstdint>
#include <string.h>

/*
|时间戳    |time_stamp  |uint32_t |整数 |4个字节|
|悬停     |Hover        |uint8    |整数 |1个字节|
|位置     |XYZ_POS      |uint8    |整数 |1个字节|
|速度     |XYZ_VEL      |uint8    |整数 |1个字节|
|定高移动 |XY_VEL_Z_POS |uint8    |整数 |1个字节|
|姿态     |XYZ_ATT      |uint8    |整数 |1个字节|
|轨迹     |TRAJECTORY   |uint8    |整数 |1个字节|
*/
enum class ControlType : uint8_t
{
    Hover = 1,
    XYZ_POS,
    XYZ_VEL,
    XY_VEL_Z_POS,
    XYZ_ATT,
    TRAJECTORY
};

/*
|时间戳   |time_stamp |uint32_t |整数 |4个字节|
|板外模式 |Offboard   |uint8    |整数 |1个字节|
|定点    |Position    |uint8    |整数 |1个字节|
|悬停    |Hold        |uint8    |整数 |1个字节|
|自稳    |Stabilized  |uint8    |整数 |1个字节|
|手动    |Manual      |uint8    |整数 |1个字节|
*/
enum class ModeType : uint8_t
{
    Offboard = 1,
    Position,
    Hold,
    Stabilized,
    Manual
};

enum class MessageType : uint8_t
{
    HeartBeatMSG = 1,
    STATUSMSG,
    MODEMSG,
    TAKEOFFMSG,
    CONTROLMSG
};

/*
|时间戳 |time_stamp |uint32_t |整数 |4个字节|
|起飞   |takeoff    |uint8    |整数 |1个字节|
|降落   |land       |uint8    |整数 |1个字节|
*/
struct takeoff_msg
{
    uint32_t time_stamp;
    uint8_t takeoff;

    void assign(uint32_t time_stamp, uint8_t takeoff)
    {
        this->time_stamp = time_stamp;
        this->takeoff = takeoff;
    }
};

struct mode_msg
{
    uint32_t time_stamp;
    uint8_t uav_mode;

    void assign(uint32_t time_stamp, uint8_t uav_mode)
    {
        this->time_stamp = time_stamp;
        this->uav_mode = uav_mode;
    }
};

/*
|时间戳 |time_stamp |uint32_t |整数|
|类型   |type       |uint8_t  |整数
|X      |x          |float32  |正负 3位小数|
|Y      |y          |float32  |正负 3位小数|
|Z      |z          |float32  |正负 3位小数|
|偏航角 |yaw        |float32  |正负 3位小数[-180,180] |
|横滚角 |roll       |float32  |正负 3位小数[-90,90]|
|俯仰角 |pich       |float32  |正负 3位小数[-90,90]|
|角速度 |yaw_rate   |bool     |布尔|
|参考系 |frame      |uint8    |整数 |1个字节|
*/
struct control_msg
{
    uint32_t time_stamp;
    uint8_t type;
    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pich;
    bool yaw_rate;
    uint8_t frame;

    void assign(uint32_t time_stamp, uint8_t type, float x, float y, float z, float yaw, float roll, float pich, bool yaw_rate, uint8_t frame)
    {
        this->time_stamp = time_stamp;
        this->type = type;
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
        this->roll = roll;
        this->pich = pich;
        this->yaw_rate = yaw_rate;
        this->frame = frame;
    }
};

/*
|HEAD     |LENGTH     |MSG_ID   |ROBOT_ID  |PAYLOAD |CHECK|
|消息帧头 |数据帧长度  |消息编号 |机器人编号 |数据帧   |校验和|
|2字节    |4字节      |1字节    |1字节     |自定义   |2字节|
|int8     |uint32     |uint8    |uint8     |自定义   |uint16|

*/
struct Message
{
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint8_t msg_mode;
    uint16_t check;
    union
    {
        struct takeoff_msg takeoff;
        struct mode_msg mode;
        struct control_msg control;
    } payload;
};

/*
回复数据不带数据帧
*/
struct replyMessage
{
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint8_t msg_mode;
    uint16_t check;

    void assign(uint16_t head, uint32_t length, uint8_t msg_id, uint8_t robot_id, uint8_t msg_mode)
    {
        this->head = head;
        this->length = length;
        this->msg_id = msg_id;
        this->robot_id = robot_id;
        this->msg_mode = msg_mode;
    }

    inline bool operator==(const replyMessage &v1) const
    {
        if (v1.head == head && v1.length == length && v1.msg_id == msg_id && v1.robot_id == robot_id && v1.msg_mode == msg_mode && v1.check == check)
            return true;
        else
            return false;
    }
};

/*
|时间戳 |time_stamp  |uint32_t |整数|
|编号   |uav_id      |uint8    |整数|
|机电量 |uav_battary |float32  |正数 [0.0, 1.0]|
|模式   |uav_mode    |uint8    |整数|
|位置   |uav_pos     |uint8    |整数|
|速度   |uav_vel     |float32  |正负 3位小数|
|加速度 |uav_acc     |float32  |正负 3位小数|
|姿态   |uav_att     |float32  |正负 3位小数[-180,180] |
*/
struct status_msg
{
    uint32_t time_stamp;
    uint8_t uav_id;
    float uav_battary;
    uint8_t uav_mode;
    uint8_t uav_pos;
    float uav_vel;
    float uav_acc;
    float uav_att;

    void assign(uint32_t time_stamp, uint8_t uav_id, float uav_battary, uint8_t uav_mode, uint8_t uav_pos, float uav_vel, float uav_acc, float uav_att)
    {
        this->time_stamp = time_stamp;
        this->uav_id = uav_id;
        this->uav_battary = uav_battary;
        this->uav_mode = uav_mode;
        this->uav_pos = uav_pos;
        this->uav_vel = uav_vel;
        this->uav_acc = uav_acc;
        this->uav_att = uav_att;
    }
};

uint16_t calculateMessageCheck(const Message &msg)
{
    uint16_t checksum = 0;
    uint16_t temp = 0;

    // HEAD (2 bytes)
    temp = (temp + msg.head) % 255;
    checksum = (checksum + temp) % 255;

    // LENGTH (4 bytes)
    uint32_t length = msg.length;
    for (int i = 0; i < 4; i++)
    {
        temp = (temp + (length >> (i * 8))) % 255;
        checksum = (checksum + temp) % 255;
    }

    // MSG_ID (1 byte)
    temp = (temp + msg.msg_id) % 255;
    checksum = (checksum + temp) % 255;

    // ROBOT_ID (1 byte)
    temp = (temp + msg.robot_id) % 255;
    checksum = (checksum + temp) % 255;

    // MSG_MODE (1 byte)
    temp = (temp + msg.msg_mode) % 255;
    checksum = (checksum + temp) % 255;

    // PAYLOAD (variable length)
    switch (static_cast<int>(msg.msg_id))
    {
    case static_cast<int>(MessageType::TAKEOFFMSG):
    {
        const takeoff_msg &payload = msg.payload.takeoff;
        temp = (temp + payload.time_stamp) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + payload.takeoff) % 255;
        checksum = (checksum + temp) % 255;
    }
    break;
    case static_cast<int>(MessageType::MODEMSG):
    {
        const mode_msg &payload = msg.payload.mode;
        temp = (temp + payload.time_stamp) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + payload.uav_mode) % 255;
        checksum = (checksum + temp) % 255;
    }
    break;
    case static_cast<int>(MessageType::CONTROLMSG):
    {
        const control_msg &payload = msg.payload.control;
        temp = (temp + payload.time_stamp) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + payload.type) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.x) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.y) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.z) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.yaw) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.roll) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + *(uint8_t *)&payload.pich) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + payload.yaw_rate) % 255;
        checksum = (checksum + temp) % 255;
        temp = (temp + payload.frame) % 255;
        checksum = (checksum + temp) % 255;
    }
    break;
    default:
        break;
    }

    return checksum;
}


uint16_t calculateReplyMessageCheck(const replyMessage &msg)
{
    uint16_t checksum = 0;
    uint16_t temp = 0;

    // HEAD (2 bytes)
    temp = (temp + msg.head) % 255;
    checksum = (checksum + temp) % 255;

    // LENGTH (4 bytes)
    uint32_t length = msg.length;
    for (int i = 0; i < 4; i++)
    {
        temp = (temp + (length >> (i * 8))) % 255;
        checksum = (checksum + temp) % 255;
    }

    // MSG_ID (1 byte)
    temp = (temp + msg.msg_id) % 255;
    checksum = (checksum + temp) % 255;

    // ROBOT_ID (1 byte)
    temp = (temp + msg.robot_id) % 255;
    checksum = (checksum + temp) % 255;

    // MSG_MODE (1 byte)
    temp = (temp + msg.msg_mode) % 255;
    checksum = (checksum + temp) % 255;


    return checksum;
}