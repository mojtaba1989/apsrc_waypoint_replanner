#ifndef APSRC_WAYPOINT_REPLANNER_Nl_H
#define APSRC_WAYPOINT_REPLANNER_Nl_H

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
#include <apsrc_msgs/CommandAccomplished.h>
#include <apsrc_msgs/CommandReceived.h>
#include <raptor_dbw_msgs/DriverInputReport.h>
#include <raptor_dbw_msgs/TurnSignal.h>



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
  void closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id);
  void baseWaypointsCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints);
  void driverInputCallback(const raptor_dbw_msgs::DriverInputreport::ConstPtr& msg);

  // UDP server callback for received message
  std::vector<uint8_t> handleServerResponse(const std::vector<uint8_t>& received_payload);
  std::vector<uint8_t> UDPVelocityModify(DWPMod::RequestMsgs request, ros::Time stamp); // msg_type:2
  std::vector<uint8_t> UDPPositionModify(DWPMod::RequestMsgs request, ros::Time stamp); // msg_type:3
  std::vector<uint8_t> UDPVelocityVecModify(DWPMod::RequestMsgs request, ros::Time stamp); // msg_type:4
  std::vector<uint8_t> UDPPositionVecModify(DWPMod::RequestMsgs request, ros::Time stamp); // msg_type:5
  std::vector<uint8_t> UDPReset(DWPMod::RequestMsgs request, ros::Time stamp); // msg_type:255

  // Util functions
  bool startServer();

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher mod_waypoints_pub_;
  ros::Publisher udp_request_pub_;
  ros::Publisher udp_report_pub_;


  // Subscribers
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber current_velocity_sub_, closest_waypoint_sub_;
  ros::Subscriber driver_input_sub_;


  // Internal state
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  std::mutex waypoints_mtx_;
  std::mutex status_data_mtx_;
  bool udp_server_running_ = false;
  bool received_base_waypoints_ = false;
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane original_waypoints_;
  ros::Time last_msg_time_ = ros::Time::now();

  // Empty udp message
  std::vector<uint8_t> empty_udp_msg_;

  // Current velocity of the vehicle (mm/s)
  uint16_t current_velocity_ = 0;
  ros::Time current_velocity_rcvd_time_;

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

  // Waypoint Message ID;
  uint8_t msg_id_ = 0;

  // Time gap
  double time_gap_ = 0.5;

  // 1m lateral transition time (s) (Normalized to 1m)
  double lateral_transition_duration_ = 2.0;
  double lateral_transition_rate_ = 1 / lateral_transition_duration_;

  // Driver Input Control
  bool turn_signal_is_ready_ = false;
};

}  // namespace apsrc_waypoint_replanner
#endif  // APSRC_WAYPOINT_REPLANNER_Nl_H
