import sys
import socket
from struct import *
import zlib
import time

class req_msg:
    def __init__(self, request_id):
        self.msg_id = (0, )
        self.request_id = (request_id, )
        self.reserved = (0, ) * 10
    def pack(self):
        b_msg = pack('!12b', *list(self.msg_id + self.request_id + self.reserved))
        crc = pack('I', int(hex(zlib.crc32(b_msg) & 0xffffffff), base=16))
        return b_msg + crc

socket_address = ('127.0.0.1', 1551)
msg = req_msg(1)

if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    s.sendto(msg.pack(), socket_address)
    print(msg.pack())
    data, address = s.recvfrom(4096)
    idx = 0
    msg_id, type, num_wps, fist_wp_id = unpack('>BBBi', data[idx:idx+7])
    idx = idx+7
    print("(msg_id, type, num_wps, fist_wp_id) =", (msg_id, type, num_wps, fist_wp_id))
    for i in range(num_wps):
        wp_id, x, y, z, yaw, velocity, change_flag = unpack("ifffffi", data[idx:idx+28])
        print("(wp_id, x, y, z, yaw, velocity, change_flag) = ", (wp_id, x, y, z, yaw, velocity, change_flag))
        idx = idx+28
    s.close

