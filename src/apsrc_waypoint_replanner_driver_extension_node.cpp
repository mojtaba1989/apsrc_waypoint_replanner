#include "ros/ros.h"
#include <raptor_dbw_msgs/DriverInputReport.h>
#include <raptor_dbw_msgs/TurnSignal.h>
#include <apsrc_msgs/DriverInputCommand.h>
#include <cmath>

enum class driverCMD
{
  UNKNOWN,
  NONE,
  LEFT_LANE_CHANGE,
  RIGHT_LANE_CHANGE,
  SPEED_UP,
  SPEED_DOWN,
  TIME_GAP_INCREASE,
  TIME_GAP_DECREASE,
  RESET_WAYPOINTS
};

enum class interState
{
  UNKNOWN,
  READY,
  WAIT,
  NONE_AFTER_WAIT,
  EXECUTE
};

struct driverInputState
{
  driverCMD potential_command = driverCMD::UNKNOWN;
  driverCMD previous_command = driverCMD::UNKNOWN;
  interState current_state = interState::UNKNOWN;
  bool double_push = true;

  void clear() {
    potential_command = driverCMD::UNKNOWN;
    previous_command = driverCMD::UNKNOWN;
    current_state = interState::UNKNOWN;
    double_push = true;
  }

  std::string report_cmd() {
    std::string txt;
    switch (potential_command)
    {
    case driverCMD::UNKNOWN :
      txt = "UNKNOWN";
      break;
    case driverCMD::LEFT_LANE_CHANGE :
      txt = "LEFT_LANE_CHANGE";
      break;
    case driverCMD::RIGHT_LANE_CHANGE :
      txt = "RIGHT_LANE_CHANGE";
      break;
    case driverCMD::SPEED_DOWN :
      txt = "SPEED_DOWN";
      break;
    case driverCMD::SPEED_UP :
      txt = "SPEED_UP";
      break;
    case driverCMD::TIME_GAP_DECREASE :
      txt = "TIME_GAP_DECREASE";
      break;
    case driverCMD::TIME_GAP_INCREASE :
      txt = "TIME_GAP_INCREASE";
      break;
    case driverCMD::RESET_WAYPOINTS :
      txt = "RESET_WAYPOINTS";
      break;
    case driverCMD::NONE :
      txt = "NONE";
      break;
    }
    return txt;
  }

  std::string report_state() {
    std::string txt;
    switch (current_state)
    {
    case interState::UNKNOWN:
      txt = "UNKNOWN";
      break;      
    case interState::READY:
      txt = "READY";
      break;    
    case interState::WAIT:
      txt = "WAIT";
      break;
    default:
      break;    
    }
    return txt;
  }
};

driverCMD extract_cmd(const raptor_dbw_msgs::DriverInputReport::ConstPtr& msg)
{
  uint8_t pushed = 0;
  driverCMD new_cmd;
  if (msg->turn_signal.value == raptor_dbw_msgs::TurnSignal::LEFT){
    pushed++;
    new_cmd = driverCMD::LEFT_LANE_CHANGE;
  }

  if (msg->turn_signal.value == raptor_dbw_msgs::TurnSignal::RIGHT){
    pushed++;
    new_cmd = driverCMD::RIGHT_LANE_CHANGE;
  }

  if (msg->adaptive_cruise_decrease_distance_button){
    pushed++;
    new_cmd = driverCMD::TIME_GAP_DECREASE;
  }

  if (msg->adaptive_cruise_increase_distance_button){
    pushed++;
    new_cmd = driverCMD::TIME_GAP_INCREASE;
  }

  if (msg->cruise_accel_button){
    pushed++;
    new_cmd = driverCMD::SPEED_UP;
  }

  if (msg->cruise_decel_button){
    pushed++;
    new_cmd = driverCMD::SPEED_DOWN;
  }

  if (msg->cruise_resume_button){
    pushed++;
    new_cmd = driverCMD::RESET_WAYPOINTS;
  }

  if (pushed == 0){
    new_cmd = driverCMD::NONE; 
  } else if (pushed > 1)
  {
    new_cmd = driverCMD::UNKNOWN;
  }
  return new_cmd;
}

class wpReplanner_ext {
public:
  wpReplanner_ext(){
    nh_ = ros::NodeHandle();
    driver_input_sub_ = nh_.subscribe("/vehicle/driver_input_report", 1, &wpReplanner_ext::driverInputCallback, this);
    driver_input_pub_ = nh_.advertise<apsrc_msgs::DriverInputCommand>("waypoint_replanner/driver_command", 1, true);   
  }

  void driverInputCallback(const raptor_dbw_msgs::DriverInputReport::ConstPtr& msg)
  {
    driverCMD new_cmd = extract_cmd(msg);
    
    switch (new_cmd)
    {
    case driverCMD::UNKNOWN :
      break;
    case driverCMD::LEFT_LANE_CHANGE :
      driver_input_state_.double_push = false;
      break;
    case driverCMD::RIGHT_LANE_CHANGE :
      driver_input_state_.double_push = false;
      break;
    case driverCMD::SPEED_DOWN :
      driver_input_state_.double_push = true;
      break;
    case driverCMD::SPEED_UP :
      driver_input_state_.double_push = true;
      break;
    case driverCMD::TIME_GAP_DECREASE :
      driver_input_state_.double_push = true;
      break;
    case driverCMD::TIME_GAP_INCREASE :
      driver_input_state_.double_push = true;
      break;
    case driverCMD::RESET_WAYPOINTS :
      driver_input_state_.double_push = true;
      break;
    default:
      break;
    }

    if (driver_input_state_.previous_command == new_cmd){
      return;
    }

    if (new_cmd == driverCMD::UNKNOWN){
      driver_input_state_.clear();
      return;
    }

    interState next_state = interState::UNKNOWN;
    switch (driver_input_state_.current_state)
    {
    case interState::UNKNOWN:
      next_state = new_cmd == driverCMD::NONE ? interState::READY : interState::UNKNOWN;
      break;
    case interState::READY:
      driver_input_state_.potential_command = new_cmd;
      next_state = driver_input_state_.double_push ? interState::WAIT : interState::EXECUTE;
      break;
    case interState::WAIT:
      next_state = new_cmd == driverCMD::NONE ? interState::NONE_AFTER_WAIT : interState::UNKNOWN;
      break;
    case interState::NONE_AFTER_WAIT:
      next_state = new_cmd == driver_input_state_.potential_command ? interState::EXECUTE : interState::UNKNOWN;
      break;
    case interState::EXECUTE:
      switch (new_cmd)
      {
      case driverCMD::NONE:
        next_state = driver_input_state_.double_push ? interState::NONE_AFTER_WAIT : interState::READY;
        break;
      default:
        next_state = interState::UNKNOWN;
        break;
      }
    }

    switch (next_state)
    {
    case interState::UNKNOWN:
      driver_input_state_.clear();
      return;

    case interState::READY:
      driver_input_state_.current_state = interState::READY;
      driver_input_state_.previous_command = new_cmd;
      return;

    case interState::WAIT:
      driver_input_state_.current_state = interState::WAIT;
      driver_input_state_.previous_command = new_cmd;
      ROS_INFO("Press again to execute command %s", driver_input_state_.report_cmd().c_str());
      return;

    case interState::NONE_AFTER_WAIT:
      driver_input_state_.current_state = interState::NONE_AFTER_WAIT;
      driver_input_state_.previous_command = new_cmd;
      return;

    case interState::EXECUTE:
      driver_input_state_.current_state = interState::EXECUTE;
      driver_input_state_.previous_command = new_cmd;
      apsrc_msgs::DriverInputCommand msg;
      
      msg.header.seq = seq_;
      seq_++;
      msg.header.stamp = ros::Time::now();

      switch (driver_input_state_.potential_command)
      {
      case driverCMD::RIGHT_LANE_CHANGE:
        msg.value = apsrc_msgs::DriverInputCommand::RIGHT_LANE_CHANGE;
        break;
      case driverCMD::LEFT_LANE_CHANGE:
        msg.value = apsrc_msgs::DriverInputCommand::LEFT_LANE_CHANGE;
        break;
      case driverCMD::SPEED_UP:
        msg.value = apsrc_msgs::DriverInputCommand::SPEED_UP;
        break;
      case driverCMD::SPEED_DOWN:
        msg.value = apsrc_msgs::DriverInputCommand::SPEED_DOWN;
        break;
      case driverCMD::TIME_GAP_INCREASE:
        msg.value = apsrc_msgs::DriverInputCommand::TIME_GAP_INCREASE;
        break;
      case driverCMD::TIME_GAP_DECREASE:
        msg.value = apsrc_msgs::DriverInputCommand::TIME_GAP_DECREASE;
        break;
      case driverCMD::RESET_WAYPOINTS:
        msg.value = apsrc_msgs::DriverInputCommand::RESET_WAYPOINTS;
        break;
      default:
        break;
      }

      driver_input_pub_.publish(msg);
      break;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber driver_input_sub_;
  ros::Publisher driver_input_pub_;

  driverInputState driver_input_state_;
  uint32_t seq_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apsrc_waypoint_replanner_driver_extension_node");
    wpReplanner_ext driver_ext;
    ros::spin();
    return 0;
}
