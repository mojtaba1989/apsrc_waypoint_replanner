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
            b_msg = b_msg + pack('=i2BfB89x', *list(args))
        elif self.request_id == 3:
            b_msg = b_msg + pack('=i2B2fB85x', *list(args))
        elif self.request_id == 4:
            b_msg = b_msg + pack('100x')
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
        self.service_processing_time_us = 1
        self.msg = msg

    def header_unpack(self, prnt=True):
        unpacked_header = unpack('=2B3I3Bf3x', self.msg[0:24])
        self.msg_id = unpacked_header[0]
        self.request_id = unpacked_header[1]
        self.time = unpacked_header[2]
        self.time_ns = unpacked_header[3]
        self.data_size_bytes = unpacked_header[4]
        self.service_msg_id = unpacked_header[5]
        self.service_request_id = unpacked_header[6]
        self.service_accomplished = unpacked_header[7]
        self.service_processing_time_us = unpacked_header[8]
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y in zip(list(vars(self).keys())[:-1],
                                                             list(vars(self).values())[:-1])))
        else:
            print(unpacked_header[8])

    def unpack(self, prnt=True):
            self.header_unpack(prnt)
            if self.request_id == 1:
                num_waypoints, first_global_waypoint_id = unpack('=BI', self.msg[24:29])
                if prnt:
                    print("\n".join("{} : {}".format(x, y) for x, y in zip(list(['num_waypoints', 'first_global_waypoint_id'])
                                                                     , list([num_waypoints, first_global_waypoint_id]))))
                    columns = ['wp_id', 'x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
                    table = []
                    idx = 29
                    for _ in range(num_waypoints):
                        table.append(list(unpack("ifffffi", self.msg[idx:idx+28])))
                        idx = idx+28
                    df = pd.DataFrame(table, columns=columns)
                    print(df)

            elif self.request_id == 4:
                data_r = unpack('=i2H', self.msg[24:32])
                header = ['closest_global_waypoint_id', 'target_global_velocity', 'current_velocity']
                if prnt:
                    print("\n".join("{} : {}".format(x, y) for x, y in zip(list(header), list(data_r))))
                else:
                    return list(data_r)[0]




socket_address = ('127.0.0.1', 1551)


if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    msg = ReqMsg(1)
    # s.sendto(msg.pack(), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # reply.unpack()

    # msg.request_id = 4
    # for _ in range(100):
    #     time.sleep(1)
    #     s.sendto(msg.pack(), socket_address)
    #     data, address = s.recvfrom(4096)
    #     reply = ReqMsgUnpack(data)
    #     reply.unpack()


    # msg.request_id = 2
    # time.sleep(.1)
    # s.sendto(msg.pack(40, 10, 1, 0, 1), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # reply.unpack()
    #
    # msg.request_id = 4
    # time.sleep(.1)
    # s.sendto(msg.pack(), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # id = reply.unpack(False)
    # while id < 40:
    #     time.sleep(1)
    #     s.sendto(msg.pack(), socket_address)
    #     data, address = s.recvfrom(4096)
    #     reply = ReqMsgUnpack(data)
    #     id = reply.unpack(False)
    #
    # msg.request_id = 2
    # time.sleep(15)
    # s.sendto(msg.pack(40, 10, 1, 3, 1), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # reply.unpack()

    # msg.request_id = 3
    # time.sleep(.1)
    # s.sendto(msg.pack(5, 10, 0, 1, 1, 0), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # reply.unpack()

    msg.request_id = 1
    for _ in range(100):
        time.sleep(.1)
        s.sendto(msg.pack(), socket_address)
        data, address = s.recvfrom(4096)
        reply = ReqMsgUnpack(data)
        reply.unpack(False)


    # msg.request_id = 2
    # time.sleep(.1)
    # s.sendto(msg.pack(38, 15, 0, -1.5, 1), socket_address)
    # data, address = s.recvfrom(4096)
    # reply = ReqMsgUnpack(data)
    # reply.unpack()

    s.close()
