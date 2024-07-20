#include <cstdint>
#include <string.h>

/*
    move mode
*/
#define CONTROL_TYPE_HOVER 1
#define CONTROL_TYPE_XYZ_POS 2
#define CONTROL_TYPE_XYZ_VEL 3
#define CONTROL_TYPE_XY_VEL_Z_POS 4
#define CONTROL_TYPE_XYZ_ATT 5
#define CONTROL_TYPE_TRAJECTORY 6

/*
 fly mode
*/
#define MODE_TYPE_OFFBOARD 1
#define MODE_TYPE_POSITION 2
#define MODE_TYPE_HOLD 3
#define MODE_TYPE_STABILIZED 4
#define MODE_TYPE_MANUAL 5

/*
    MESSAGE ID
*/
#define HEARTBEAT_MESSAGE 1
#define STATUS_MESSAGE 2
#define TAKEOFF_MESSAGE 101
#define CONTROL_MESSAGE 102
#define MODE_MESSAGE 103

/*
    size of struct
*/
#define SIZE_HEARTBEAT 8
#define SIZE_TAKEOFF 15
#define SIZE_MODE 15
#define SIZE_CONTROL 39
#define SIZE_STATUS 63

// 
struct takeoff_msg {
    uint32_t time_stamp; /**< @param time_stamp timestamp */
    uint8_t takeoff; /**< @param takeoff takeoff flag */
};

struct mode_msg {
    uint32_t time_stamp; /**< @param time_stamp timestamp */
    uint8_t uav_mode; /**< @param uav_mode UAV mode */
};

struct control_msg {
    uint32_t time_stamp; /**< @param time_stamp timestamp */
    uint8_t type; /**< @param type control type */
    float x; /**< @param x x-coordinate */
    float y; /**< @param y y-coordinate */
    float z; /**< @param z z-coordinate */
    float yaw; /**< @param yaw yaw angle */
    float roll; /**< @param roll roll angle */
    float pich; /**< @param pich pitch angle */
    bool yaw_rate; /**< @param yaw_rate yaw rate flag */
    uint8_t frame; /**< @param frame reference frame */
};

struct status_msg {
    uint32_t time_stamp; /**< @param time_stamp timestamp */
    uint8_t uav_id; /**< @param uav_id UAV ID */
    uint8_t uav_battary; /**< @param uav_battary UAV battery level */
    uint8_t uav_mode; /**< @param uav_mode UAV mode */
    float uav_pos_x; /**< @param uav_pos_x x-coordinate */
    float uav_pos_y; /**< @param uav_pos_y y-coordinate */
    float uav_pos_z; /**< @param uav_pos_z z-coordinate */
    float uav_vel_x; /**< @param uav_vel_x x-velocity */
    float uav_vel_y; /**< @param uav_vel_y y-velocity */
    float uav_vel_z; /**< @param uav_vel_z z-velocity */
    float uav_acc_x; /**< @param uav_acc_x x-acceleration */
    float uav_acc_y; /**< @param uav_acc_y y-acceleration */
    float uav_acc_z; /**< @param uav_acc_z z-acceleration */
    float uav_att_r; /**< @param uav_att_r roll angle */
    float uav_att_p; /**< @param uav_att_p pitch angle */
    float uav_att_y; /**< @param uav_att_y yaw angle */
};

// Message structures
struct Heartbeat_Message {
    int32_t count; /**< @param head message header */
    uint32_t time_stamp; /**< @param head message header */
};

// Message structures
struct Takeoff_Message {
    uint16_t head; /**< @param head message header */
    uint32_t length; /**< @param length message length */
    uint8_t msg_id; /**< @param msg_id message ID */
    uint8_t robot_id; /**< @param robot_id robot ID */
    uint16_t check; /**< @param check checksum */
    takeoff_msg payload; /**< @param payload takeoff message payload */
};

struct Mode_Message {
    uint16_t head; /**< @param head message header */
    uint32_t length; /**< @param length message length */
    uint8_t msg_id; /**< @param msg_id message ID */
    uint8_t robot_id; /**< @param robot_id robot ID */
    uint16_t check; /**< @param check checksum */
    mode_msg payload; /**< @param payload mode message payload */
};


struct Control_Message {
    uint16_t head; /**< @param head message header */
    uint32_t length; /**< @param length message length */
    uint8_t msg_id; /**< @param msg_id message ID */
    uint8_t robot_id; /**< @param robot_id robot ID */
    uint16_t check; /**< @param check checksum */
    control_msg payload; /**< @param payload control message payload */
};



struct Status_Message {
    uint16_t head; /**< @param head message header */
    uint32_t length; /**< @param length message length */
    uint8_t msg_id; /**< @param msg_id message ID */
    uint8_t robot_id; /**< @param robot_id robot ID */
    uint16_t check; /**< @param check checksum */
    status_msg payload; /**< @param payload status message payload */
};

char* pack_Heartbeat_Message(int32_t count, uint32_t time_stamp) {
    static char buf[sizeof(Heartbeat_Message)];
    memcpy(buf, &count, 4);
    memcpy(buf + 4, &time_stamp, 4);
    return buf;
}

char* encode_Heartbeat_Message(const Heartbeat_Message* msg) {
    static char buf[sizeof(Heartbeat_Message)];
    memcpy(buf, &msg->count, 4);
    memcpy(buf + 4, &msg->time_stamp, 4);
    return buf;
}

char* pack_Takeoff_Message(uint16_t head, uint32_t length, uint8_t msg_id, uint8_t robot_id, uint16_t check, takeoff_msg payload) {
    static char buf[sizeof(Takeoff_Message)];
    memcpy(buf, &head, 2);
    memcpy(buf + 2, &length, 4);
    memcpy(buf + 6, &msg_id, 1);
    memcpy(buf + 7, &robot_id, 1);
    memcpy(buf + 8, &check, 2);
    memcpy(buf + 10, &payload.time_stamp, 4);
    memcpy(buf + 14, &payload.takeoff, 1);
    return buf;
}

char* encode_Takeoff_Message(const Takeoff_Message* msg) {
    static char buf[sizeof(Takeoff_Message)];
    memcpy(buf, &msg->head, 2);
    memcpy(buf + 2, &msg->length, 4);
    memcpy(buf + 6, &msg->msg_id, 1);
    memcpy(buf + 7, &msg->robot_id, 1);
    memcpy(buf + 8, &msg->check, 2);
    memcpy(buf + 10, &msg->payload.time_stamp, 4);
    memcpy(buf + 14, &msg->payload.takeoff, 1);
    return buf;
}

char* pack_Mode_Message(uint16_t head, uint32_t length, uint8_t msg_id, uint8_t robot_id, uint16_t check, mode_msg payload) {
    static char buf[sizeof(Mode_Message)];
    memcpy(buf, &head, 2);
    memcpy(buf + 2, &length, 4);
    memcpy(buf + 6, &msg_id, 1);
    memcpy(buf + 7, &robot_id, 1);
    memcpy(buf + 8, &check, 2);
    memcpy(buf + 10, &payload.time_stamp, 4);
    memcpy(buf + 14, &payload.uav_mode, 1);
    return buf;
}

char* encode_Mode_Message(const Mode_Message* msg) {
    static char buf[sizeof(Mode_Message)];
    memcpy(buf, &msg->head, 2);
    memcpy(buf + 2, &msg->length, 4);
    memcpy(buf + 6, &msg->msg_id, 1);
    memcpy(buf + 7, &msg->robot_id, 1);
    memcpy(buf + 8, &msg->check, 2);
    memcpy(buf + 10, &msg->payload.time_stamp, 4);
    memcpy(buf + 14, &msg->payload.uav_mode, 1);
    return buf;
}

char* pack_Control_Message(uint16_t head, uint32_t length, uint8_t msg_id, uint8_t robot_id, uint16_t check, control_msg payload) {
    static char buf[sizeof(Control_Message)];
    memcpy(buf, &head, 2);
    memcpy(buf + 2, &length, 4);
    memcpy(buf + 6, &msg_id, 1);
    memcpy(buf + 7, &robot_id, 1);
    memcpy(buf + 8, &check, 2);
    memcpy(buf + 10, &payload.time_stamp, 4);
    memcpy(buf + 14, &payload.type, 1);
    memcpy(buf + 15, &payload.x, 4);
    memcpy(buf + 19, &payload.y, 4);
    memcpy(buf + 23, &payload.z, 4);
    memcpy(buf + 27, &payload.yaw, 4);
    memcpy(buf + 31, &payload.roll, 4);
    memcpy(buf + 35, &payload.pich, 4);
    memcpy(buf + 39, &payload.yaw_rate, 1);
    memcpy(buf + 40, &payload.frame, 1);
    return buf;
}

char* encode_Control_Message(const Control_Message* msg) {
    static char buf[sizeof(Control_Message)];
    memcpy(buf, &msg->head, 2);
    memcpy(buf + 2, &msg->length, 4);
    memcpy(buf + 6, &msg->msg_id, 1);
    memcpy(buf + 7, &msg->robot_id, 1);
    memcpy(buf + 8, &msg->check, 2);
    memcpy(buf + 10, &msg->payload.time_stamp, 4);
    memcpy(buf + 14, &msg->payload.type, 1);
    memcpy(buf + 15, &msg->payload.x, 4);
    memcpy(buf + 19, &msg->payload.y, 4);
    memcpy(buf + 23, &msg->payload.z, 4);
    memcpy(buf + 27, &msg->payload.yaw, 4);
    memcpy(buf + 31, &msg->payload.roll, 4);
    memcpy(buf + 35, &msg->payload.pich, 4);
    memcpy(buf + 39, &msg->payload.yaw_rate, 1);
    memcpy(buf + 40, &msg->payload.frame, 1);
    return buf;
}

char* pack_Status_Message(uint16_t head, uint32_t length, uint8_t msg_id, uint8_t robot_id, uint16_t check, status_msg payload) {
    static char buf[sizeof(Status_Message)];
    memcpy(buf, &head, 2);
    memcpy(buf + 2, &length, 4);
    memcpy(buf + 6, &msg_id, 1);
    memcpy(buf + 7, &robot_id, 1);
    memcpy(buf + 8, &check, 2);
    memcpy(buf + 10, &payload.time_stamp, 4);
    memcpy(buf + 14, &payload.uav_id, 1);
    memcpy(buf + 15, &payload.uav_battary, 1);
    memcpy(buf + 16, &payload.uav_mode, 1);
    memcpy(buf + 17, &payload.uav_pos_x, 4);
    memcpy(buf + 21, &payload.uav_pos_y, 4);
    memcpy(buf + 25, &payload.uav_pos_z, 4);
    memcpy(buf + 29, &payload.uav_vel_x, 4);
    memcpy(buf + 33, &payload.uav_vel_y, 4);
    memcpy(buf + 37, &payload.uav_vel_z, 4);
    memcpy(buf + 41, &payload.uav_acc_x, 4);
    memcpy(buf + 45, &payload.uav_acc_y, 4);
    memcpy(buf + 49, &payload.uav_acc_z, 4);
    memcpy(buf + 53, &payload.uav_att_r, 4);
    memcpy(buf + 57, &payload.uav_att_p, 4);
    memcpy(buf + 61, &payload.uav_att_y, 4);
    return buf;
}

char* encode_Status_Message(const Status_Message* msg) {
    static char buf[sizeof(Status_Message)];
    memcpy(buf, &msg->head, 2);
    memcpy(buf + 2, &msg->length, 4);
    memcpy(buf + 6, &msg->msg_id, 1);
    memcpy(buf + 7, &msg->robot_id, 1);
    memcpy(buf + 8, &msg->check, 2);
    memcpy(buf + 10, &msg->payload.time_stamp, 4);
    memcpy(buf + 14, &msg->payload.uav_id, 1);
    memcpy(buf + 15, &msg->payload.uav_battary, 1);
    memcpy(buf + 16, &msg->payload.uav_mode, 1);
    memcpy(buf + 17, &msg->payload.uav_pos_x, 4);
    memcpy(buf + 21, &msg->payload.uav_pos_y, 4);
    memcpy(buf + 25, &msg->payload.uav_pos_z, 4);
    memcpy(buf + 29, &msg->payload.uav_vel_x, 4);
    memcpy(buf + 33, &msg->payload.uav_vel_y, 4);
    memcpy(buf + 37, &msg->payload.uav_vel_z, 4);
    memcpy(buf + 41, &msg->payload.uav_acc_x, 4);
    memcpy(buf + 45, &msg->payload.uav_acc_y, 4);
    memcpy(buf + 49, &msg->payload.uav_acc_z, 4);
    memcpy(buf + 53, &msg->payload.uav_att_r, 4);
    memcpy(buf + 57, &msg->payload.uav_att_p, 4);
    memcpy(buf + 61, &msg->payload.uav_att_y, 4);
    return buf;
}

void unpack_Heartbeat_Message(char* buf, Heartbeat_Message* msg) {
    memcpy(&msg->count, buf, 4);
    memcpy(&msg->time_stamp, buf + 4, 4);
}

void unpack_Takeoff_Message(char* buf, Takeoff_Message* msg) {
    memcpy(&msg->head, buf, 2);
    memcpy(&msg->length, buf + 2, 4);
    memcpy(&msg->msg_id, buf + 6, 1);
    memcpy(&msg->robot_id, buf + 7, 1);
    memcpy(&msg->check, buf + 8, 2);
    memcpy(&msg->payload.time_stamp, buf + 10, 4);
    memcpy(&msg->payload.takeoff, buf + 14, 1);
}

void unpack_Mode_Message(char* buf, Mode_Message* msg) {
    memcpy(&msg->head, buf, 2);
    memcpy(&msg->length, buf + 2, 4);
    memcpy(&msg->msg_id, buf + 6, 1);
    memcpy(&msg->robot_id, buf + 7, 1);
    memcpy(&msg->check, buf + 8, 2);
    memcpy(&msg->payload.time_stamp, buf + 10, 4);
    memcpy(&msg->payload.uav_mode, buf + 14, 1);
}


void unpack_Control_Message(char* buf, Control_Message* msg) {
    memcpy(&msg->head, buf, 2);
    memcpy(&msg->length, buf + 2, 4);
    memcpy(&msg->msg_id, buf + 6, 1);
    memcpy(&msg->robot_id, buf + 7, 1);
    memcpy(&msg->check, buf + 8, 2);
    memcpy(&msg->payload.time_stamp, buf + 10, 4);
    memcpy(&msg->payload.type, buf + 14, 1);
    memcpy(&msg->payload.x, buf + 15, 4);
    memcpy(&msg->payload.y, buf + 19, 4);
    memcpy(&msg->payload.z, buf + 23, 4);
    memcpy(&msg->payload.yaw, buf + 27, 4);
    memcpy(&msg->payload.roll, buf + 31, 4);
    memcpy(&msg->payload.pich, buf + 35, 4);
    memcpy(&msg->payload.yaw_rate, buf + 39, 1);
    memcpy(&msg->payload.frame, buf + 40, 1);
}


void unpack_Status_Message(char* buf, Status_Message* msg){
    memcpy(&msg->head, buf, 2);
    memcpy(&msg->length, buf + 2, 4);
    memcpy(&msg->msg_id, buf + 6, 1);
    memcpy(&msg->robot_id, buf + 7, 1);
    memcpy(&msg->check, buf + 8, 2);
    memcpy(&msg->payload.time_stamp, buf + 10, 4);
    memcpy(&msg->payload.uav_id, buf + 14, 1);
    memcpy(&msg->payload.uav_battary, buf + 15, 1);
    memcpy(&msg->payload.uav_mode, buf + 16, 1);
    memcpy(&msg->payload.uav_pos_x, buf + 17, 4);
    memcpy(&msg->payload.uav_pos_y, buf + 21, 4);
    memcpy(&msg->payload.uav_pos_z, buf + 25, 4);
    memcpy(&msg->payload.uav_vel_x, buf + 29, 4);
    memcpy(&msg->payload.uav_vel_y, buf + 33, 4);
    memcpy(&msg->payload.uav_vel_z, buf + 37, 4);
    memcpy(&msg->payload.uav_acc_x, buf + 41, 4);
    memcpy(&msg->payload.uav_acc_y, buf + 45, 4);
    memcpy(&msg->payload.uav_acc_z, buf + 49, 4);
    memcpy(&msg->payload.uav_att_r, buf + 53, 4);
    memcpy(&msg->payload.uav_att_p, buf + 57, 4);
    memcpy(&msg->payload.uav_att_y, buf + 61, 4);
}

bool check_header_and_length(const char* buf, size_t length) {
    if (length < 10) { 
        return false;
    }
    uint16_t head = *(uint16_t*)buf; 
    uint32_t length_field = *(uint32_t*)(buf + 2); 
    return head == 0XADFF && length_field == length - 2; 
}

// 
bool check_message(const char* buf, size_t length, uint8_t expected_msg_id, uint8_t expected_robot_id) {
    if (!check_header_and_length(buf, length)) {
        return false;
    }
    uint8_t msg_id = buf[6]; 
    uint8_t robot_id = buf[7]; 
    if (msg_id != expected_msg_id || robot_id != expected_robot_id) {
        return false;
    }
    return true;
}


void extract_message_header(const char* buf, uint16_t* head, uint32_t* length, uint8_t* msg_id, uint8_t* robot_id, uint16_t* check) {
    memcpy(head, buf, 2);
    memcpy(length, buf + 2, 4);
    memcpy(msg_id, buf + 6, 1);
    memcpy(robot_id, buf + 7, 1);
    memcpy(check, buf + 8, 2);
}


// Calculate checksum for Takeoff_Message
uint16_t calculate_checksum_Takeoff_Message(const Takeoff_Message* msg) {
    uint32_t checksum = 0;
    checksum += msg->head;
    checksum += msg->length;
    checksum += msg->msg_id;
    checksum += msg->robot_id;
    checksum += msg->payload.time_stamp;
    checksum += msg->payload.takeoff;
    checksum %= (1 << 16); // Limit the checksum to 16 bits
    return (uint16_t)checksum;
}

// Calculate checksum for Mode_Message
uint16_t calculate_checksum_Mode_Message(const Mode_Message* msg) {
    uint32_t checksum = 0;
    checksum += msg->head;
    checksum += msg->length;
    checksum += msg->msg_id;
    checksum += msg->robot_id;
    checksum += msg->payload.time_stamp;
    checksum += msg->payload.uav_mode;
    return (uint16_t)checksum;
}

// Calculate checksum for Control_Message
uint16_t calculate_checksum_Control_Message(const Control_Message* msg) {
    uint32_t checksum = 0;
    checksum += msg->head;
    checksum += msg->length;
    checksum += msg->msg_id;
    checksum += msg->robot_id;
    checksum += msg->payload.time_stamp;
    checksum += msg->payload.type;
    checksum += *(uint32_t*)&msg->payload.x;
    checksum += *(uint32_t*)&msg->payload.y;
    checksum += *(uint32_t*)&msg->payload.z;
    checksum += *(uint32_t*)&msg->payload.yaw;
    checksum += *(uint32_t*)&msg->payload.roll;
    checksum += *(uint32_t*)&msg->payload.pich;
    checksum += msg->payload.yaw_rate;
    checksum += msg->payload.frame;
    return (uint16_t)checksum;
}

// Calculate checksum for Status_Message
uint16_t calculate_checksum_Status_Message(const Status_Message* msg) {
    uint32_t checksum = 0;
    checksum += msg->head;
    checksum += msg->length;
    checksum += msg->msg_id;
    checksum += msg->robot_id;
    checksum += msg->payload.time_stamp;
    checksum += msg->payload.uav_id;
    checksum += msg->payload.uav_battary;
    checksum += msg->payload.uav_mode;
    checksum += *(uint32_t*)&msg->payload.uav_pos_x;
    checksum += *(uint32_t*)&msg->payload.uav_pos_y;
    checksum += *(uint32_t*)&msg->payload.uav_pos_z;
    checksum += *(uint32_t*)&msg->payload.uav_vel_x;
    checksum += *(uint32_t*)&msg->payload.uav_vel_y;
    checksum += *(uint32_t*)&msg->payload.uav_vel_z;
    checksum += *(uint32_t*)&msg->payload.uav_acc_x;
    checksum += *(uint32_t*)&msg->payload.uav_acc_y;
    checksum += *(uint32_t*)&msg->payload.uav_acc_z;
    checksum += *(uint32_t*)&msg->payload.uav_att_r;
    checksum += *(uint32_t*)&msg->payload.uav_att_p;
    checksum += *(uint32_t*)&msg->payload.uav_att_y;
    return (uint16_t)checksum;
}