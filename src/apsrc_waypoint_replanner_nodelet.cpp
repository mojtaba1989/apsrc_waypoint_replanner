#include <string>
#include <vector>


#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_nodelet.hpp"
#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_plugins.hpp"


namespace apsrc_waypoint_replanner
{
ApsrcWaypointReplannerNl::ApsrcWaypointReplannerNl()
{
}

ApsrcWaypointReplannerNl::~ApsrcWaypointReplannerNl()
{
}

void ApsrcWaypointReplannerNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  mod_waypoints_pub_  = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 10, true);
  eco_cruise_pub_     = nh_.advertise<apsrc_msgs::AvpCommand>("/lead_vehicle/command", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcWaypointReplannerNl::velocityCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &ApsrcWaypointReplannerNl::closestWaypointCallback, this);
  base_waypoints_sub_   = nh_.subscribe("base_waypoints", 1, &ApsrcWaypointReplannerNl::baseWaypointsCallback, this);
  driver_input_sub_     = nh_.subscribe("waypoint_replanner/driver_command", 1, &ApsrcWaypointReplannerNl::driverInputCallback, this);

  // MABx Subscribers
  vel_sub_      = nh_.subscribe("/mabx_commands/velocity_cmd", 1, &ApsrcWaypointReplannerNl::velocityCmdCallback, this);
  pos_sub_      = nh_.subscribe("/mabx_commands/position_cmd", 1, &ApsrcWaypointReplannerNl::positionCmdCallback, this);
  vel_arr_sub_  = nh_.subscribe("/mabx_commands/velocity_array_cmd", 1, &ApsrcWaypointReplannerNl::velocityArrayCmdCallback, this);
  pos_arr_sub_  = nh_.subscribe("/mabx_commands/position_array_cmd", 1, &ApsrcWaypointReplannerNl::positionArrayCmdCallback, this);
  reset_sub_    = nh_.subscribe("/mabx_commands/reset_cmd", 1, &ApsrcWaypointReplannerNl::resetCmdCallback, this);

  // Longitudinal gap control
  lng_gap_ctrl_.lead_speed_based_ctr_enb = false;
  lng_gap_ctrl_.stop_distance = STOP_DISTANCE_;
  lng_gap_ctrl_.time_gap = LNG_GAP_MAX_;
}

void ApsrcWaypointReplannerNl::loadParams()
{
  pnh_.param("lateral_transition_duration", lateral_transition_duration_, 2.0);
  lateral_transition_rate_ = 1 / lateral_transition_duration_;

  pnh_.param("LANE_WIDTH", LANE_WIDTH_, 3.2);
  pnh_.param("LNG_GAP_MIN", LNG_GAP_MIN_, 2.5);
  pnh_.param("LNG_GAP_MAX", LNG_GAP_MAX_, 1.0);
  pnh_.param("LNG_GAP_STEP", LNG_GAP_STEP_, 0.5);
  pnh_.param("STOP_DISTANCE", STOP_DISTANCE_, 11.0);
  pnh_.param("SPEED_MOD_STEP", SPEED_MOD_STEP_, 0.44704);

  ROS_INFO("Parameters Loaded");
}

void ApsrcWaypointReplannerNl::velocityCmdCallback(const apsrc_msgs::VelocityCommand::ConstPtr& msg)
{
  if (!received_base_waypoints_)
  {
    return;
  }

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t begin_id = msg->waypoint_id;
  if (msg->waypoint_id < 0)
  {
    begin_id = std::min(last_id, closest_waypoint_id_ - msg->waypoint_id);
  }

  
  int32_t max_count = msg->number_of_waypoints==-1 ? last_id +1 : msg->number_of_waypoints;
  double target_velocity = (msg->magnitude < 0 && msg->action == apsrc_msgs::VelocityCommand::ACTION_SET) ? 0 : msg->magnitude;
  switch (msg->unit) {
    case apsrc_msgs::VelocityCommand::UNIT_MPS : //m/s -> m/s
      break;
    case apsrc_msgs::VelocityCommand::UNIT_KMPH : //km/h -> m/s
      target_velocity /= 3.6;
      break;
    case apsrc_msgs::VelocityCommand::UNIT_MPH://mph -> m/s
      target_velocity *= 0.44704;
      break;
  }

  {
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    int32_t idx = 0;
    if (msg->action == apsrc_msgs::VelocityCommand::ACTION_SET)
    {
      while (idx < max_count && (begin_id + idx) < last_id)
      {
        base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x = target_velocity;
        idx++;
      }
    } else if (msg->action == apsrc_msgs::VelocityCommand::ACTION_MODIFTY)
    {
      while (idx < max_count && (begin_id + idx) < last_id)
      {
        base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x += target_velocity;
        if (base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x < 0)
        {
          base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x = 0;
        }
        idx++;
      }
    }
    mod_waypoints_pub_.publish(base_waypoints_);
    ROS_INFO("Following Waypoints velocity has been set/modifued: %s->%s",
             std::to_string(begin_id).c_str(),
             std::to_string(begin_id + idx - 1).c_str());
  }
  return;
}

void ApsrcWaypointReplannerNl::positionCmdCallback(const apsrc_msgs::PositionCommand::ConstPtr& msg)
{
  if (!received_base_waypoints_)
  {
    return;
  }

  if (msg->action != apsrc_msgs::PositionCommand::ACTION_MODIFTY)
  {
    ROS_ERROR("This function only supports MODIFY action");
    return;
  }

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t begin_id = msg->waypoint_id;
  if (msg->waypoint_id < 0)
  {
    begin_id = std::min(last_id, closest_waypoint_id_ - msg->waypoint_id);
  }

  if (msg->number_of_waypoints!=-1){
    ROS_INFO("Num of WP forced to -1 due to safety reasons!");
  }

  int32_t max_count = last_id +1;
  double lat_shift = msg->direction;
  switch (msg->unit) {
    case apsrc_msgs::PositionCommand::UNIT_M : // m->m
      break;
    case apsrc_msgs::PositionCommand::UNIT_CM : // cm -> m
      lat_shift /= 100;
      break;
    case apsrc_msgs::PositionCommand::UNIT_INCH: //inch -> m
      lat_shift *= 0.0254;
      break;
  }

  int32_t idx = 0;
  if (msg->smoothingEn == apsrc_msgs::PositionCommand::SMOOTHING_ENABLE){
    std::vector<double> lat_shift_vector = awp_plugins::lane_change_lateral_shift(lat_shift, A_MAX_COMFORT_, J_MAX_COMFORT_, current_velocity_);
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    while(idx < lat_shift_vector.size() && (begin_id + idx) <= last_id){
      base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, begin_id + idx, lat_shift_vector[idx]);
      ++idx;
    }
    while (idx < max_count && (begin_id + idx) <= last_id){
      base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, begin_id + idx, lat_shift);
      ++idx;
    }
  } else if (msg->smoothingEn == apsrc_msgs::PositionCommand::SMOOTHING_DISABLE){
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    idx = 0;
    while(begin_id + idx <= last_id){
      base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, begin_id + idx, lat_shift);
      ++idx;
    }
  } else {
    ROS_ERROR("Requested Manual Smoothing Mode is depreciated!");
    return;
  }

  ROS_INFO("Following Waypoints position has been modified: %s->%s",
            std::to_string(begin_id).c_str(),
            std::to_string(begin_id + idx - 1).c_str()
          );
  mod_waypoints_pub_.publish(base_waypoints_);
  return;
}

void ApsrcWaypointReplannerNl::velocityArrayCmdCallback(const apsrc_msgs::VelocityArrayCommand::ConstPtr& msg)
{
  if (!received_base_waypoints_)
  {
    return;
  }

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t begin_id = msg->waypoint_id;
  if (msg->waypoint_id < 0)
  {
    begin_id = std::min(last_id, closest_waypoint_id_ - msg->waypoint_id);
  }

  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
  int32_t idx = 0;
  while (idx < msg->num_of_waypoints && (begin_id + idx) < last_id)
  {
    base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x = msg->velocity_vector[idx];
    idx++;
  }

  while ((begin_id + idx) <= last_id)
  {
    base_waypoints_.waypoints[begin_id + idx].twist.twist.linear.x = msg->velocity_vector[msg->num_of_waypoints-1];
    idx++;
  }

  
  ROS_INFO("Following Waypoints velocity has been set using vector: %s->%s",
            std::to_string(begin_id).c_str(),
            std::to_string(begin_id + idx - 1).c_str()
          );
  mod_waypoints_pub_.publish(base_waypoints_);
  return;
}

void ApsrcWaypointReplannerNl::positionArrayCmdCallback(const apsrc_msgs::PositionArrayCommand::ConstPtr& msg)
{
  if (!received_base_waypoints_)
  {
    return;
  }

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t begin_id = msg->waypoint_id;
  if (msg->waypoint_id < 0)
  {
    begin_id = std::min(last_id, closest_waypoint_id_ - msg->waypoint_id);
  }

  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
  int32_t idx = 0;
  while(idx < msg->num_of_waypoints && (begin_id + idx) < last_id){
    base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_,
                                                 begin_id + idx,
                                                 msg->lat_shift_vector[idx]);
    ++idx;
  }

  while((begin_id + idx) < last_id){
    base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_,
                                                 begin_id + idx,
                                                 msg->lat_shift_vector[msg->num_of_waypoints-1]);
    ++idx;
  }

  ROS_INFO("Following Waypoints position has been modified using vector: %s->%s",
            std::to_string(begin_id).c_str(),
            std::to_string(begin_id + idx - 1).c_str()
          );
  mod_waypoints_pub_.publish(base_waypoints_);
  return;
}

void ApsrcWaypointReplannerNl::resetCmdCallback(const apsrc_msgs::DriverInputCommand::ConstPtr& msg)
{
  if (msg->value == apsrc_msgs::DriverInputCommand::RESET_WAYPOINTS)
  {
    if (received_base_waypoints_)
    {
      base_waypoints_ = original_waypoints_;
      mod_waypoints_pub_.publish(base_waypoints_);
      ROS_INFO("Waypoints have been reset!");
    }
  }
}

void ApsrcWaypointReplannerNl::driverInputCallback(const apsrc_msgs::DriverInputCommand::ConstPtr& cmd)
{
  switch (cmd->value)
  {
  case apsrc_msgs::DriverInputCommand::RESET_WAYPOINTS:
    if (received_base_waypoints_){
      base_waypoints_ = original_waypoints_;
      mod_waypoints_pub_.publish(base_waypoints_);
      ROS_INFO("Waypoints have been reset!");
    }
    return;

  case apsrc_msgs::DriverInputCommand::LEFT_LANE_CHANGE:
    if (received_base_waypoints_ && closest_waypoint_id_){
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t start_id = closest_waypoint_id_ + 5;
      std::vector<double> lat_shift_vector = awp_plugins::lane_change_lateral_shift(-LANE_WIDTH_, A_MAX_COMFORT_, J_MAX_COMFORT_, current_velocity_);
      if (last_id - start_id < lat_shift_vector.size()){
        ROS_WARN("Not enough waypoints available for safe lane change!");
        return;
      }
      if (blind_spot_not_clear_left) {
        if (ros::Time::now() - blind_spot_update_ < ros::Duration(1.0)) {
          ROS_WARN("Blind spot not clear!");
          return;
        } else {
          blind_spot_update_ = ros::Time::now();
          blind_spot_not_clear_left = false;
          blind_spot_not_clear_right = false;
        }
      }
      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      int32_t idx = 0;
      while(idx < lat_shift_vector.size() && (start_id + idx) <= last_id){
        base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, start_id + idx, lat_shift_vector[idx]);
        ++idx;
      }
      while (start_id + idx <= last_id){
        base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, start_id + idx, -LANE_WIDTH_);
        ++idx;
      }
      ROS_INFO("Lane Change Command starting at %s", std::to_string(start_id).c_str());
      mod_waypoints_pub_.publish(base_waypoints_);
    }
    return;

  case apsrc_msgs::DriverInputCommand::RIGHT_LANE_CHANGE:
    if (received_base_waypoints_ && closest_waypoint_id_){
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t start_id = closest_waypoint_id_ + 5;
      std::vector<double> lat_shift_vector = awp_plugins::lane_change_lateral_shift(LANE_WIDTH_, A_MAX_COMFORT_, J_MAX_COMFORT_, current_velocity_);
      if (last_id - start_id < lat_shift_vector.size()){
        ROS_WARN("Not enough waypoints available for safe lane change!");
        return;
      }
      if (blind_spot_not_clear_right) {
        if (ros::Time::now() - blind_spot_update_ < ros::Duration(1.0)) {
          ROS_WARN("Blind spot not clear!");
          return;
        } else {
          blind_spot_update_ = ros::Time::now();
          blind_spot_not_clear_left = false;
          blind_spot_not_clear_right = false;
        }
      }
      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      int32_t idx = 0;
      while(idx < lat_shift_vector.size() && (start_id + idx) <= last_id){
        base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, start_id + idx, lat_shift_vector[idx]);
        ++idx;
      }
      while (start_id + idx <= last_id){
        base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, start_id + idx, LANE_WIDTH_);
        ++idx;
      }
      ROS_INFO("Lane Change Command starting at %s", std::to_string(start_id).c_str());
      mod_waypoints_pub_.publish(base_waypoints_);
    }
    return;

  case apsrc_msgs::DriverInputCommand::TIME_GAP_INCREASE:
    if (lng_gap_ctrl_.time_gap <= LNG_GAP_MAX_ - LNG_GAP_STEP_)
    {
      lng_gap_ctrl_.time_gap += LNG_GAP_STEP_;
      ROS_INFO("Longitudianl Gap Increased to %s", std::to_string(lng_gap_ctrl_.time_gap).c_str());
    } else if (lng_gap_ctrl_.time_gap < LNG_GAP_MAX_)
    {
      lng_gap_ctrl_.time_gap = LNG_GAP_MAX_;
      ROS_INFO("Longitudianl Gap Increased to %s", std::to_string(lng_gap_ctrl_.time_gap).c_str());
    } else 
    {
      lng_gap_ctrl_.time_gap = LNG_GAP_MAX_;
      lng_gap_ctrl_.lead_speed_based_ctr_enb = false;
      ROS_INFO("Speed-based Longitudianl Gap Ctrl set OFF");
    }
    eco_cruise_pub_.publish(lng_gap_ctrl_);
    return;

  case apsrc_msgs::DriverInputCommand::TIME_GAP_DECREASE:
    if (!lng_gap_ctrl_.lead_speed_based_ctr_enb){
      lng_gap_ctrl_.time_gap = LNG_GAP_MAX_;
      lng_gap_ctrl_.lead_speed_based_ctr_enb = true;
      ROS_INFO("Speed-based Longitudianl Gap Ctrl set ON");
    } else if (lng_gap_ctrl_.time_gap >= LNG_GAP_MIN_ + LNG_GAP_STEP_)
    {
      lng_gap_ctrl_.time_gap -= LNG_GAP_STEP_;
      ROS_INFO("Longitudianl Gap Decreased to %s", std::to_string(lng_gap_ctrl_.time_gap).c_str());
    } else if (lng_gap_ctrl_.time_gap > LNG_GAP_MIN_)
    {
      lng_gap_ctrl_.time_gap = LNG_GAP_MIN_;
      ROS_INFO("Longitudianl Gap Dencrease to %s", std::to_string(lng_gap_ctrl_.time_gap).c_str());
    } else
    {
      ROS_WARN("Smaller Longitudianl Gap is NOT Allowed!");
    }
    eco_cruise_pub_.publish(lng_gap_ctrl_);
    return;

  case apsrc_msgs::DriverInputCommand::SPEED_UP:
    if (received_base_waypoints_ && closest_waypoint_id_){
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t start_id = closest_waypoint_id_;
      if (last_id - start_id <= 5){
        ROS_WARN("Not enough waypoints available for speed set!");
        return;
      }
      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      double target_vel = current_velocity_ + SPEED_MOD_STEP_;
      for (int32_t i = start_id; i < last_id; i++) {
        base_waypoints_.waypoints[i].twist.twist.linear.x = target_vel;
      }
      
      ROS_INFO("Speed Increased to %s", std::to_string(target_vel).c_str());
      mod_waypoints_pub_.publish(base_waypoints_);
    }
    return;

  case apsrc_msgs::DriverInputCommand::SPEED_DOWN:
    if (received_base_waypoints_ && closest_waypoint_id_){
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t start_id = closest_waypoint_id_;
      if (last_id - start_id <= 5){
        ROS_WARN("Not enough waypoints available for speed set!");
        return;
      }
      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      double zero = 0;
      double target_vel = std::max(zero, current_velocity_ - SPEED_MOD_STEP_);
      for (int32_t i = start_id; i < last_id; i++) {
        base_waypoints_.waypoints[i].twist.twist.linear.x = target_vel;
      }
      
      ROS_INFO("Speed Decreased to %s", std::to_string(target_vel).c_str());
      mod_waypoints_pub_.publish(base_waypoints_);
    }
    return;
  }
}

void ApsrcWaypointReplannerNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
}

void ApsrcWaypointReplannerNl::baseWaypointsCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints)
{
  if (!received_base_waypoints_)
  {
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    base_waypoints_ = *base_waypoints;
    original_waypoints_ = *base_waypoints;
    mod_waypoints_pub_.publish(base_waypoints_);
    received_base_waypoints_ = true;
  }
}

void ApsrcWaypointReplannerNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  current_velocity_ = current_velocity->twist.linear.x;
}

void ApsrcWaypointReplannerNl::blindSpotCallback(const apsrc_msgs::BlindSpotChecker::ConstPtr& msg)
{
  blind_spot_update_ = msg->header.stamp;
  blind_spot_not_clear_left = msg->left;
  blind_spot_not_clear_right = msg->right; 
}

}  // namespace apsrc_waypoint_replanner
PLUGINLIB_EXPORT_CLASS(apsrc_waypoint_replanner::ApsrcWaypointReplannerNl,
                       nodelet::Nodelet);
