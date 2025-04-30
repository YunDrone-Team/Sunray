import socket
import threading
import struct
from message import State_Message

class TcpSocketHandler:
    def __init__(self, server_address):
        self.server_address = server_address
        self.sock = self.connect_to_server()

    def connect_to_server(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(self.server_address)
            print("TCP连接成功")
            return sock
        except:
            print("TCP连接失败")
            return None
        


class UdpReceiver:
    def __init__(self, server_address):
        self.server_address = server_address
        self.sock = self.connect_to_server()
        self.recv_flag = False
        self.msg = None
        self.state = State_Message()
        if self.sock:
            self.start_receiving()

    def connect_to_server(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(self.server_address)
            print("UDP连接成功")
            return sock
        except:
            print("UDP连接失败")
            return None

    def decode_State_Message(self):
        # format_string = '<HIBBH'
        format_string = '<HIBBHIBBB15sBB14fB'
        
        while True:
            data, addr = self.sock.recvfrom(1024)
            data = data[:91]
            self.msg = struct.unpack(format_string, data)
            # print(self.msg)
            self.state.copy_from(self.msg)
            self.recv_flag = True
            # print(self.state)

    def start_receiving(self):
        thread = threading.Thread(target=self.decode_State_Message)
        thread.daemon = True
        thread.start()

if __name__ == '__main__':
    # udp
    udp_server_address = ('192.168.0.15', 8968)
    udp_receiver = UdpReceiver(udp_server_address)
    # udp_receiver.decode_State_Message()
