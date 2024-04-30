#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>


namespace DWPMod {
struct smoothing_ctrl_t {
  uint8_t beginning; //1:fix steps 2: fix number
  uint8_t ending; //1:fix steps 2: fix number
  float beginning_smoothing_extra;
  float ending__smoothing_extra;
}; // 10 bytes

struct velocityCMD_t {
  int32_t waypoint_id = 0;
  int32_t number_of_waypoints = 0;
  uint8_t action = 0; //modify(0) set(1)
  float magnitude = 0; // velocity in mps for unit=0
  uint8_t unit = 0; // m/s(0) km/h (1) mph (2)
  uint8_t smoothingEn = 0; // disable(0) enable(1) manual(2)
  struct smoothing_ctrl_t smoothingCtrl;
}; //15 or 25 bytes

struct positionCMD_t {
  int32_t waypoint_id = 0;
  int32_t number_of_waypoints = 0;
  uint8_t action = 0; //modify(0) add(1) remove(2)
  float direction; //lateral in meters/ + for right/ - for left
  uint8_t unit = 0; // m(0) cm(1) inch(2)
  uint8_t smoothingEn = 0; // disable(0) enable(1) manual(2)
  struct smoothing_ctrl_t smoothingCtrl;
}; // 15 or 25 bytes

struct velocityProfileCMD_t
{
  int32_t waypoint_id;
  uint8_t num_of_waypoints;
  float velocity_vector[50];
}; // 205 bytes

struct positionProfileCMD_t
{
  int32_t waypoint_id;
  uint8_t num_of_waypoints;
  float lat_shift_vector[50];
}; // 205 bytes

class header {//24 bytes
public:
  uint8_t msg_id;
  uint8_t request_id;
  int32_t time_stamp[2];
  uint32_t data_size_byte;
  uint8_t data_info[10];

  int unpack(const std::vector<uint8_t> &buffer) {
    msg_id = buffer[0];
    request_id = buffer[1];
    std::memcpy(&time_stamp, &buffer[2], 8);
    std::memcpy(&data_size_byte, &buffer[10], 4);
    std::memcpy(&data_info, &buffer[14], 10);
    return 24;
  }
};

class velocityCMD {
public:
  struct velocityCMD_t cmd;

  bool unpack(const std::vector<uint8_t> &buffer, int idx) {
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.number_of_waypoints, &buffer[idx+4], 4);
    std::memcpy(&cmd.action, &buffer[idx+8], 1);
    std::memcpy(&cmd.magnitude, &buffer[idx+9], 4);
    std::memcpy(&cmd.unit, &buffer[idx+13], 1);
    std::memcpy(&cmd.smoothingEn, &buffer[idx+14], 1);
    if (cmd.smoothingEn == 2){
      std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[idx+15], 1);
      std::memcpy(&cmd.smoothingCtrl.ending, &buffer[idx+16], 4);
      std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[idx+20], 1);
      std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[idx+21], 4);
      return idx+25;
    }
    return idx+15;
  }
};

class positionCMD {
public:
  struct positionCMD_t cmd;

  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.number_of_waypoints, &buffer[idx+4], 4);
    std::memcpy(&cmd.action, &buffer[idx+8], 1);
    switch (cmd.action) {
      case 0: std::memcpy(&cmd.direction, &buffer[idx+9], 4);
    }
    std::memcpy(&cmd.unit, &buffer[idx+13], 1);
    std::memcpy(&cmd.smoothingEn, &buffer[idx+14], 1);
    if (cmd.smoothingEn == 2){
      std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[idx+15], 1);
      std::memcpy(&cmd.smoothingCtrl.ending, &buffer[idx+16], 4);
      std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[idx+20], 1);
      std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[idx+21], 4);
      return idx+25;
    }
    return idx+15;
  }
};

class veocityVectorCMD {
public:
  struct velocityProfileCMD_t cmd;
  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.num_of_waypoints, &buffer[idx+4], 1);
    std::memcpy(&cmd.velocity_vector, &buffer[idx+5], 200);
    return idx+205;
  }
};

class positionVectorCMD {
public:
  struct positionProfileCMD_t cmd;
  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.num_of_waypoints, &buffer[idx+4], 1);
    std::memcpy(&cmd.lat_shift_vector, &buffer[idx+5], 200);
  return idx+205;
  }
};

class RequestMsgs { // 256 bytes
public:
  DWPMod::header header;
  velocityCMD velocityCmd;
  positionCMD positionCmd;
  positionVectorCMD positionVectorCmd;
  veocityVectorCMD velocityVectorCmd;
  uint8_t reserved[33];
  uint32_t crc;

  bool unpack(const std::vector<uint8_t> &buffer) {
    int idx = header.unpack(buffer);
    switch (header.request_id) {
      case 1: //waypoints report
        break;
      case 2:
        idx = velocityCmd.unpack(buffer, idx); //velocity CMD
        break;
      case 3:
        idx = positionCmd.unpack(buffer, idx); //position CMD
        break;
      case 4:
        idx = velocityVectorCmd.unpack(buffer, idx); //velocity vector CMD
        break;
      case 5:
        idx = positionVectorCmd.unpack(buffer, idx); //position vector CMD
        break;
      case 255: // reset CMD
        break;
    }
    std::memcpy(&crc, &buffer[252], 4);
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 252);
    if (crc == msg_crc.checksum()) {
      return true;
    } else {
      return false;
    }
  }
};
}// namespace DWPMod

#endif  // PACKET_DEFINITIONS_H
