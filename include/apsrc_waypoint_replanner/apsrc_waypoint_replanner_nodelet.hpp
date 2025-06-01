#ifndef APSRC_WAYPOINT_REPLANNER_Nl_H
#define APSRC_WAYPOINT_REPLANNER_Nl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <apsrc_msgs/CommandAccomplished.h>
#include <apsrc_msgs/CommandReceived.h>
#include <apsrc_msgs/AvpCommand.h>
#include <apsrc_msgs/DriverInputCommand.h>
#include <apsrc_msgs/VelocityCommand.h>
#include <apsrc_msgs/PositionCommand.h>
#include <apsrc_msgs/VelocityArrayCommand.h>
#include <apsrc_msgs/PositionArrayCommand.h>
#include <apsrc_msgs/BlindSpotChecker.h>
#include <raptor_dbw_msgs/DriverInputReport.h>
#include <raptor_dbw_msgs/TurnSignal.h>

const ros::Duration OUTDATED_DATA_TIMEOUT(0.5);

namespace apsrc_waypoint_replanner
{

struct wp_based_report {
  size_t waypoint;
  apsrc_msgs::CommandAccomplished report;
};

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
  void driverInputCallback(const apsrc_msgs::DriverInputCommand::ConstPtr& cmd);

  // MABx Subscriber callbacks
  void velocityCmdCallback(const apsrc_msgs::VelocityCommand::ConstPtr& msg);
  void positionCmdCallback(const apsrc_msgs::PositionCommand::ConstPtr& msg);
  void velocityArrayCmdCallback(const apsrc_msgs::VelocityArrayCommand::ConstPtr& msg);
  void positionArrayCmdCallback(const apsrc_msgs::PositionArrayCommand::ConstPtr& msg);
  void resetCmdCallback(const apsrc_msgs::DriverInputCommand::ConstPtr& msg);
  void blindSpotCallback(const apsrc_msgs::BlindSpotChecker::ConstPtr& msg);

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher mod_waypoints_pub_;
  ros::Publisher eco_cruise_pub_;


  // Subscribers
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber current_velocity_sub_, closest_waypoint_sub_;
  ros::Subscriber driver_input_sub_;
  ros::Subscriber blind_spot_sub_;

  // MABx Subscribers
  ros::Subscriber vel_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber vel_arr_sub_;
  ros::Subscriber pos_arr_sub_;
  ros::Subscriber reset_sub_;

  // Internal state
  std::thread udp_server_thread_;
  std::mutex waypoints_mtx_;
  std::mutex status_data_mtx_;
  bool received_base_waypoints_ = false;
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane original_waypoints_;

  // Current velocity of the vehicle (m/s)
  double current_velocity_ = 0;

  // Closest global waypoint id
  int32_t closest_waypoint_id_ = -1;


  // 1m lateral transition time (s) (Normalized to 1m)
  double lateral_transition_duration_ = 2.0;
  double lateral_transition_rate_ = 1 / lateral_transition_duration_;

  double A_MAX_COMFORT_ = 1.5;
  double J_MAX_COMFORT_ = 1.0;

  // Driver Input Commands config
  double LANE_WIDTH_ = 3.2;

  apsrc_msgs::AvpCommand lng_gap_ctrl_;
  double LNG_GAP_MAX_ = 2.5;
  double LNG_GAP_MIN_ = 1.0;
  double LNG_GAP_STEP_ = 0.5;
  double STOP_DISTANCE_ = 11.0;

  double SPEED_MOD_STEP_ = 0.44704;

  // Blind Spot checker for virtual vehicles
  bool blind_spot_not_clear_left = false;
  bool blind_spot_not_clear_right = false;
  ros::Time blind_spot_update_ = ros::Time::now();
};

}  // namespace apsrc_waypoint_replanner
#endif  // APSRC_WAYPOINT_REPLANNER_Nl_H
