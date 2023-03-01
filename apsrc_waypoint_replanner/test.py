import sys
import socket
from struct import *
import zlib
import time
import pandas as pd

def flatten_tuple(nested_tuple):
    flat_tuple = ()
    for element in nested_tuple:
        if isinstance(element, tuple):
            flat_tuple += flatten_tuple(element)
        else:
            flat_tuple += (element,)
    return flat_tuple

class ReqMsg:
    def __init__(self, request_id):
        self.msg_id = 0
        self.request_id = request_id
        msg_time = time.time()
        self.time = int(msg_time)
        self.time_ns = int((msg_time % 1) * 1e9)
        self.data_size_bytes = 0
        self.data_info = (0, ) * 10

    def pack(self, *args):
        b_msg = pack("=2B3I10B", *list(flatten_tuple(tuple(vars(self).values()))))
        if self.request_id == 1:
            b_msg = b_msg + pack('100x')
        elif self.request_id == 2:
            b_msg = b_msg + pack('=I2Bf90x', *list(args))
        elif self.request_id == 3:
            b_msg = b_msg + pack('=I2B2f86x', *list(args))
        b_msg = b_msg + pack('I', int(hex(zlib.crc32(b_msg) & 0xffffffff), base=16))
        self.msg_id = self.msg_id + 1
        return b_msg

class ReqMsgUnpack:
    def __init__(self, msg):
        self.msg_id = 0
        self.request_id = 0
        self.time = 0
        self.time_ns = 0
        self.data_size_bytes = 0
        self.service_msg_id = 0
        self.service_request_id = 0
        self.service_accomplished = 0
        self.msg = msg

    def header_unpack(self):
        unpacked_header = unpack('=2B3I10B', self.msg[0:24])
        self.msg_id = unpacked_header[0]
        self.request_id = unpacked_header[1]
        self.time = unpacked_header[2]
        self.time_ns = unpacked_header[3]
        self.data_size_bytes = unpacked_header[4]
        self.service_msg_id = unpacked_header[5]
        self.service_request_id = unpacked_header[6]
        self.service_accomplished = unpacked_header[7]
        print("\n".join("{} : {}".format(x, y) for x, y in zip(list(vars(self).keys())[:-1],
                                                             list(vars(self).values())[:-1])))

    def unpack(self):
            self.header_unpack()
            if self.request_id == 1:
                num_waypoints, first_global_waypoint_id = unpack('=BI', self.msg[17:22])
                print("\n".join("{} : {}".format(x, y) for x, y in zip(list(['num_waypoints', 'first_global_waypoint_id'])
                                                                     , list([num_waypoints, first_global_waypoint_id]))))
                columns = ['wp_id', 'x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
                table = []
                idx = 24
                for _ in range(num_waypoints):
                    table.append(list(unpack("ifffffi", self.msg[idx:idx+28])))
                    idx = idx+28
                df = pd.DataFrame(table, columns=columns)
                print(df)




socket_address = ('127.0.0.1', 1551)


if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    msg = ReqMsg(1)
    s.sendto(msg.pack(), socket_address)
    data, address = s.recvfrom(4096)
    reply = ReqMsgUnpack(data)
    reply.unpack()


    msg.request_id = 2
    time.sleep(5)
    s.sendto(msg.pack(20, 100, 1, 3), socket_address)
    data, address = s.recvfrom(4096)
    reply = ReqMsgUnpack(data)
    reply.unpack()

    msg.request_id = 3
    time.sleep(5)
    s.sendto(msg.pack(20, 40, 0, 1, 1), socket_address)
    data, address = s.recvfrom(4096)
    reply = ReqMsgUnpack(data)
    reply.unpack()

    msg.request_id = 3
    time.sleep(5)
    s.sendto(msg.pack(100, 10, 0, -1, 0), socket_address)
    data, address = s.recvfrom(4096)
    reply = ReqMsgUnpack(data)
    reply.unpack()
    s.close()