#ifndef DVP_WAYPOINT_REPLANNER_ApsrcWaypointReplannerNl_H
#define DVP_WAYPOINT_REPLANNER_ApsrcWaypointReplannerNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <thread>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>

#include "apsrc_waypoint_replanner/packet_definitions.hpp"

const ros::Duration OUTDATED_DATA_TIMEOUT(0.5);

namespace apsrc_waypoint_replanner
{
class ApsrcWaypointReplannerNl : public nodelet::Nodelet
{
public:
  ApsrcWaypointReplannerNl();
  ~ApsrcWaypointReplannerNl();

private:
  // Init
  virtual void onInit();
  void loadParams();

  // Subscriber callbacks
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity);
  void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status);
  void closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id);
  void baseWaypointsCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints);

  // UDP server callback for new data received
  std::vector<uint8_t> handleServerResponse(const std::vector<uint8_t>& received_payload);

  // Util functions
  bool startServer();
  void generateGlobalPath(DVPMod::VelocityProfile velocity_profile);

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher max_waypoints_pub_, mod_waypoints_pub_;


  // Subscribers
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber current_velocity_sub_, vehicle_status_sub_, closest_waypoint_sub_;


  // Internal state
  AS::Network::UDPServer udp_server_;
  AS::Network::UDPInterface udp_inetrface_;
  std::thread udp_server_thread_;
  std::mutex waypoints_mtx_;
  std::mutex status_data_mtx_;
  bool udp_server_running_ = false;
  bool received_base_waypoints_ = false;
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane current_waypoints_;
  bool velocity_profile_enabled_ = false;

  // Current velocity of the vehicle (mm/s)
  uint16_t current_velocity_ = 0;
  ros::Time current_velocity_rcvd_time_;

  // DBW in manual or autonomy
  bool dbw_engaged_ = false;
  ros::Time dbw_engaged_rcvd_time_;

  // Closest global waypoint id
  int32_t closest_waypoint_id_ = 0;
  ros::Time closest_waypoint_id_rcvd_time_;

  // Velocity at the closest global waypoint (mm/s)
  uint16_t target_velocity_ = 0;

  // Parameters
  std::string server_ip_;
  int server_port_;

  // Highest allowed speed of any waypoint on the global path (km/h)
  double max_speed_;

  // Time out duration
  double time_out_;

  // Waypoint Message ID;
  uint8_t msg_id_ = 0;
};

}  // namespace dvp_waypoint_replanner
#endif  // DVP_WAYPOINT_REPLANNER_ApsrcWaypointReplannerNl_H
