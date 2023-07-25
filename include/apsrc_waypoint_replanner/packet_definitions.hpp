#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>


namespace DVPMod {
    struct waypoint_t {
        uint32_t waypoint_id = 0;
        float x = 0;
        float y = 0;
        float z = 0;
        float yaw = 0;
        float velocity = 0;
        uint32_t change_flag = 0;
    };//28 bytes

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

    class header {//24 bytes
    public:
        uint8_t msg_id;
        uint8_t request_id;
        int32_t time_stamp[2];
        ros::Time ros_time_stamp;
        uint32_t data_size_byte;
        uint8_t data_info[10];

        bool pack(std::vector<uint8_t> &buffer) {
          buffer[0] = msg_id;
          buffer[1] = request_id;
          std::memcpy(&buffer[2], &time_stamp, 8);
          std::memcpy(&buffer[10], &data_size_byte, 4);
          std::memcpy(&buffer[14], &data_info, 10);
          return true;
        }

        bool unpack(const std::vector<uint8_t> &buffer) {
          msg_id = buffer[0];
          request_id = buffer[1];
          std::memcpy(&ros_time_stamp, &buffer[2], 8);
          std::memcpy(&data_size_byte, &buffer[10], 4);
          std::memcpy(&data_info, &buffer[14], 10);
          return true;
        }
    };

    class velocityCMD {
    public:
        struct velocityCMD_t cmd;

        bool unpack(const std::vector<uint8_t> &buffer) {
          std::memcpy(&cmd.waypoint_id, &buffer[24], 4);
          std::memcpy(&cmd.number_of_waypoints, &buffer[28], 4);
          std::memcpy(&cmd.action, &buffer[32], 1);
          std::memcpy(&cmd.magnitude, &buffer[33], 4);
          std::memcpy(&cmd.unit, &buffer[37], 1);
          std::memcpy(&cmd.smoothingEn, &buffer[38], 1);
          if (cmd.smoothingEn == 2){
            std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[39], 1);
            std::memcpy(&cmd.smoothingCtrl.ending, &buffer[40], 4);
            std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[44], 1);
            std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[45], 4);
          }
        }
    };

    class positionCMD {
    public:
        struct positionCMD_t cmd;
        int unpack(const std::vector<uint8_t> &buffer){
          std::memcpy(&cmd.waypoint_id, &buffer[24], 4);
          std::memcpy(&cmd.number_of_waypoints, &buffer[28], 4);
          std::memcpy(&cmd.action, &buffer[32], 1);
          switch (cmd.action) {
            case 0: std::memcpy(&cmd.direction, &buffer[33], 4);
          }
          std::memcpy(&cmd.unit, &buffer[37], 1);
          std::memcpy(&cmd.smoothingEn, &buffer[38], 1);
          if (cmd.smoothingEn == 2){
            std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[39], 1);
            std::memcpy(&cmd.smoothingCtrl.ending, &buffer[40], 4);
            std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[44], 1);
            std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[45], 4);
          }
          return 1;
        }
    };

    class statusMsg {
    public:
        int32_t closest_global_waypoint_id;
        uint16_t target_global_velocity;
        uint16_t current_velocity;

        void pack(std::vector<uint8_t> &buffer) {
          std::memcpy(&buffer[24], &closest_global_waypoint_id, 4);
          std::memcpy(&buffer[28], &target_global_velocity, 2);
          std::memcpy(&buffer[30], &current_velocity, 2);
        }
    };

    class RequestMsgs { // 128 bytes
    public:
        DVPMod::header header;
        DVPMod::velocityCMD velocityCmd;
        DVPMod::positionCMD positionCmd;
        uint8_t reserved[100];
        uint32_t crc;

        bool unpack(const std::vector<uint8_t> &buffer) {
          header.unpack(buffer);
          switch (header.request_id) {
            case 1: //waypoints report
              break;
            case 2:
              velocityCmd.unpack(buffer); //velocity CMD
              break;
            case 3:
              positionCmd.unpack(buffer); //position CMD
              break;
            case 4: // vehicle status report
              break;
            case 255: // reset CMD
              break;
          }
          std::memcpy(reserved, &buffer[24], 100);
          std::memcpy(&crc, &buffer[124], 4);
          boost::crc_32_type msg_crc;
          msg_crc.process_bytes(&buffer[0], 124);
          if (crc == msg_crc.checksum()) {
            return true;
          } else {
            return false;
          }
        }
    };

    class WaypointsArrayMsg {// (24/2800/14)=2838 bytes
    public:
        uint8_t num_waypoints = 0;
        uint32_t first_global_waypoint_id = 0;
        struct waypoint_t waypoints_array[100];
        int data_size = 2800;

        bool pack(std::vector<uint8_t> &buffer) {
          buffer[24] = num_waypoints;
          std::memcpy(&buffer[25], &first_global_waypoint_id, 4);
          std::memcpy(&buffer[29], waypoints_array, 2800);
          return true;
        }

        void unpack(const std::vector<uint8_t> &buffer) {
          num_waypoints = buffer[24];
          std::memcpy(&first_global_waypoint_id, &buffer[25], sizeof(first_global_waypoint_id));
          std::memcpy(waypoints_array, &buffer[29], 2800);
        }
    };

    class ServiceReply {//
    public:
        DVPMod::header header;
        DVPMod::WaypointsArrayMsg waypoints_array_msg;
        DVPMod::statusMsg status_msg;
        uint8_t service_msg_id;
        uint8_t service_request_id;
        bool service_accomplished;
        float processing_time;
        uint8_t reserved[10];
        uint32_t crc;

        std::vector<uint8_t> pack() {
          std::vector<uint8_t> buffer(4096);
          header.pack(buffer);
          switch (service_request_id) {
            case 1:
              waypoints_array_msg.pack(buffer);
              break;
            case 4:
              status_msg.pack(buffer);
          }
          buffer[14] = service_msg_id;
          buffer[15] = service_request_id;
          buffer[16] = static_cast<uint8_t>(service_accomplished);
          std::memcpy(&buffer[17], &processing_time, 4);
          std::memcpy(&buffer[4082], reserved, 10);

          // Calculate CRC
          boost::crc_32_type msg_crc;
          msg_crc.process_bytes(&buffer[0], 4092);
          crc = msg_crc.checksum();
          std::memcpy(&buffer[4092], &crc, 4);
          return buffer;
        }
    };

    class VelocityProfile {
        public:
            uint8_t msg_id;
            uint8_t status;
            uint16_t first_global_waypoint_id;
            uint16_t target_velocities[50];
            uint8_t reserved[10];
            uint32_t crc;

            std::vector<uint8_t> pack() {
              std::vector<uint8_t> buffer(118);
              buffer[0] = msg_id;
              buffer[1] = status;
              std::memcpy(&buffer[2], &first_global_waypoint_id, sizeof(first_global_waypoint_id));
              std::memcpy(&buffer[4], target_velocities, sizeof(target_velocities));
              std::memcpy(&buffer[104], reserved, sizeof(reserved));

              // Calculate CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 114);
              crc = msg_crc.checksum();

              std::memcpy(&buffer[114], &crc, sizeof(crc));

              return buffer;
            }

            bool unpack(const std::vector<uint8_t> &buffer) {
              msg_id = buffer[0];
              status = buffer[1];
              std::memcpy(&first_global_waypoint_id, &buffer[2], sizeof(first_global_waypoint_id));
              std::memcpy(target_velocities, &buffer[4], sizeof(target_velocities));
              std::memcpy(reserved, &buffer[104], sizeof(reserved));
              std::memcpy(&crc, &buffer[114], sizeof(crc));

              // Confirm CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 114);
              if (crc == msg_crc.checksum()) {
                return true;
              } else {
                return false;
              }
            }
        };

    class StatusReply {
        public:
            uint8_t msg_id;
            bool data_valid;
            bool path_tracking_enabled;
            bool velocity_profile_enabled;
            uint16_t closest_global_waypoint_id;
            uint16_t target_global_velocity;
            uint16_t current_velocity;
            uint8_t reserved[10];
            uint32_t crc;

            std::vector<uint8_t> pack() {
              std::vector<uint8_t> buffer(24);
              buffer[0] = msg_id;
              buffer[1] = static_cast<uint8_t>(data_valid);
              buffer[2] = static_cast<uint8_t>(path_tracking_enabled);
              buffer[3] = static_cast<uint8_t>(velocity_profile_enabled);
              std::memcpy(&buffer[4], &closest_global_waypoint_id, sizeof(closest_global_waypoint_id));
              std::memcpy(&buffer[6], &target_global_velocity, sizeof(target_global_velocity));
              std::memcpy(&buffer[8], &current_velocity, sizeof(current_velocity));
              std::memcpy(&buffer[10], reserved, sizeof(reserved));

              // Calculate CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 20);
              crc = msg_crc.checksum();

              std::memcpy(&buffer[20], &crc, sizeof(crc));

              return buffer;
            }

            bool unpack(const std::vector<uint8_t> &buffer) {
              msg_id = buffer[0];
              data_valid = static_cast<bool>(buffer[1] & 0x01);
              path_tracking_enabled = static_cast<bool>(buffer[2] & 0x01);
              velocity_profile_enabled = static_cast<bool>(buffer[3] & 0x01);
              std::memcpy(&closest_global_waypoint_id, &buffer[4], sizeof(closest_global_waypoint_id));
              std::memcpy(&target_global_velocity, &buffer[6], sizeof(target_global_velocity));
              std::memcpy(&current_velocity, &buffer[8], sizeof(current_velocity));
              std::memcpy(reserved, &buffer[10], sizeof(reserved));
              std::memcpy(&crc, &buffer[20], sizeof(crc));

              // Confirm CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 20);
              if (crc == msg_crc.checksum()) {
                return true;
              } else {
                return false;
              }
            }
        };

    class TrackedObjects {
        public:
            uint8_t num_objects;
            int16_t x_pos[30];
            int16_t y_pos[30];
            int16_t speed[30];
            int16_t heading[30];
            uint8_t classification[30];
            uint16_t size[30];
            uint32_t id[30];
            uint8_t reserved[10];
            uint32_t crc;

            std::vector<uint8_t> pack() {
              std::vector<uint8_t> buffer(465);
              buffer[0] = num_objects;
              std::memcpy(&buffer[1], x_pos, sizeof(x_pos));
              std::memcpy(&buffer[61], y_pos, sizeof(y_pos));
              std::memcpy(&buffer[121], speed, sizeof(speed));
              std::memcpy(&buffer[181], heading, sizeof(heading));
              std::memcpy(&buffer[241], classification, sizeof(classification));
              std::memcpy(&buffer[271], size, sizeof(size));
              std::memcpy(&buffer[331], id, sizeof(id));
              std::memcpy(&buffer[451], reserved, sizeof(reserved));

              // Calculate CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 461);
              crc = msg_crc.checksum();

              std::memcpy(&buffer[461], &crc, sizeof(crc));

              return buffer;
            }

            bool unpack(const std::vector<uint8_t> &buffer) {
              num_objects = buffer[0];
              std::memcpy(x_pos, &buffer[1], sizeof(x_pos));
              std::memcpy(y_pos, &buffer[61], sizeof(y_pos));
              std::memcpy(speed, &buffer[121], sizeof(speed));
              std::memcpy(heading, &buffer[181], sizeof(heading));
              std::memcpy(classification, &buffer[241], sizeof(classification));
              std::memcpy(size, &buffer[271], sizeof(size));
              std::memcpy(id, &buffer[331], sizeof(id));
              std::memcpy(reserved, &buffer[451], sizeof(reserved));
              std::memcpy(&crc, &buffer[461], sizeof(crc));

              // Confirm CRC
              boost::crc_32_type msg_crc;
              msg_crc.process_bytes(&buffer[0], 461);
              if (crc == msg_crc.checksum()) {
                return true;
              } else {
                return false;
              }
            }
        };

}// namespace DVPMod

#endif  // PACKET_DEFINITIONS_H
