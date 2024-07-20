import socket
import struct

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a address and port
sock.bind(("0.0.0.0", 8088))  # replace with your desired port

print("Listening for UDP messages...")

while True:
    # Receive a UDP message
    data, addr = sock.recvfrom(1024)
    print("Received message from", addr)

    # Parse the message
    head = struct.unpack_from("!B", data)[0]
    length = struct.unpack_from("!b", data, 2)[0]
    msg_id = data[6]
    robot_id = data[7]
    msg_mode = data[8]

    print("HEAD:", head)
    print("LENGTH:", length)
    print("MSG_ID:", msg_id)
    print("ROBOT_ID:", robot_id)
    print("MSG_MODE:", msg_mode)

    # Parse the payload based on the message ID
    if msg_id == 3:  # CONTROLMSG
        control_msg = struct.unpack_from("!Ifff?fB", data, 9)
        print("CONTROL_MSG:")
        print("  TIMESTAMP:", control_msg[0])
        print("  TYPE:", control_msg[1])
        print("  X:", control_msg[2])
        print("  Y:", control_msg[3])
        print("  Z:", control_msg[4])
        print("  YAW:", control_msg[5])
        print("  ROLL:", control_msg[6])
        print("  PITCH:", control_msg[7])
        print("  YAW_RATE:", control_msg[8])
        print("  FRAME:", control_msg[9])

    else:
        print("Unknown message ID:", msg_id)