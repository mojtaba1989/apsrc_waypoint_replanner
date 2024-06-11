#include <string>
#include <vector>


#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_nodelet.hpp"
#include "apsrc_waypoint_replanner/packet_definitions.hpp"
#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_plugins.hpp"


namespace apsrc_waypoint_replanner
{
ApsrcWaypointReplannerNl::ApsrcWaypointReplannerNl()
{
}

ApsrcWaypointReplannerNl::~ApsrcWaypointReplannerNl()
{
  if (udp_server_running_)
  {
    udp_server_.stop();
    udp_server_thread_.join();
  }
}

void ApsrcWaypointReplannerNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  mod_waypoints_pub_  = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 10, true);
  udp_report_pub_     = nh_.advertise<apsrc_msgs::CommandAccomplished>("apsrc_udp/received_commands_report", 10, true);
  udp_request_pub_    = nh_.advertise<apsrc_msgs::CommandReceived>("apsrc_udp/received_commands", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcWaypointReplannerNl::velocityCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &ApsrcWaypointReplannerNl::closestWaypointCallback, this);
  base_waypoints_sub_   = nh_.subscribe("base_waypoints", 1, &ApsrcWaypointReplannerNl::baseWaypointsCallback, this);

  if (startServer())
  {
    udp_server_running_ = true;
    udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
  }
  else
  {
    udp_server_running_ = false;
    ros::requestShutdown();
  }
}

void ApsrcWaypointReplannerNl::loadParams()
{
  pnh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
  pnh_.param("server_port", server_port_, 1551);
  pnh_.param("max_speed", max_speed_, 60.0);
  pnh_.param("time_gap", time_gap_, 0.5);
  pnh_.param("lateral_transition_duration", lateral_transition_duration_, 2.0);
  lateral_transition_rate_ = 1 / lateral_transition_duration_;

  ROS_INFO("Parameters Loaded");
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  std::vector<uint8_t> udp_msg = {};
  DWPMod::RequestMsgs requestMsg;
  if (!requestMsg.unpack(received_payload))
  {
    ROS_WARN("Checksum doesn't match! dropping packet");
  }
  else if (requestMsg.header.request_id == 0) // Ignore request_id == 0
  {
  }
  else if((ros::Time::now() - last_msg_time_).toSec() <= time_gap_)
  {
    ROS_WARN("Possibility of redundant packet! dropping packet");
  }
  else
  {
    apsrc_msgs::CommandReceived ros_msg = {};
    ros_msg.msg_id = requestMsg.header.msg_id;
    ros_msg.request_id = requestMsg.header.request_id;
    ros_msg.request_stamp = ros::Time::now();
    udp_request_pub_.publish(ros_msg);

    switch (requestMsg.header.request_id) {
      case 1: // request for waypoints from spectra
        ROS_INFO("Received request: sharing waypoint");
        ROS_WARN("APS Waypoint Replanner Does NOT support waypoint sharing anymore!(__version__ > 2)");
        break;

      case 2: // request for velocity modification
        ROS_INFO("Received request: velocity modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPVelocityModify(requestMsg, ros_msg.request_stamp);
        last_msg_time_ = ros::Time::now();
        break;

      case 3: // request for position shift
        ROS_INFO("Received request: position modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPPositionModify(requestMsg, ros_msg.request_stamp);
        last_msg_time_ = ros::Time::now();
        break;
      
      case 4: // request for vector-based velocity modification 
        ROS_INFO("Received request: vector-based velocity modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPVelocityVecModify(requestMsg, ros_msg.request_stamp);
        last_msg_time_ = ros::Time::now();
        break;

      case 5: // request for vector-based position modification
        ROS_INFO("Received request: vector-based position modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPPositionVecModify(requestMsg, ros_msg.request_stamp);
        last_msg_time_ = ros::Time::now();
        break;

      case 255: // request for reset
        ROS_INFO("Received request: reset");
        udp_msg = ApsrcWaypointReplannerNl::UDPReset(requestMsg, ros_msg.request_stamp);
        last_msg_time_ = ros::Time::now();
        break;
    }
  }
  return udp_msg;
}

bool ApsrcWaypointReplannerNl::startServer()
{
  AS::Network::ReturnStatuses status = udp_server_.open(server_ip_, server_port_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not start UDP server: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }
  else
  {
    ROS_INFO("UDP server started at %s (%s)",server_ip_.c_str(), std::to_string(server_port_).c_str());
    udp_server_.registerReceiveHandler(
            std::bind(&ApsrcWaypointReplannerNl::handleServerResponse,
                      this, std::placeholders::_1));

    return true;
  }
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
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x) * 1000.0));
  current_velocity_rcvd_time_ = ros::Time::now();
}

void ApsrcWaypointReplannerNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
  closest_waypoint_id_rcvd_time_ = ros::Time::now();
}

void ApsrcWaypointReplannerNl::driverInputCallback(const raptor_dbw_msgs::DriverInputReport::ConstPtr& msg)
{
  if (msg->turn_signal.value == raptor_dbw_msgs::TurnSignal::None){
    turn_signal_is_ready_ = true;
    return;
  }

  if (turn_signal_is_ready_ && received_base_waypoints_ && closest_waypoint_id_){
    int32_t last_id = base_waypoints_.waypoints.size() - 1;
    int32_t start_id = closest_waypoint_id_ + 5;
    double lateral = 3.6;
    if (last_id - start_id <= 5){
      ROS_WARN("Not enough waypoints available for safe lane change!");
      turn_signal_is_ready_ = false;
      return;
    }

    switch (msg->turn_signal.value)
    {
    case raptor_dbw_msgs::TurnSignal::LEFT:
      lateral = -3.6;
      break;
    case raptor_dbw_msgs::TurnSignal::RIGHT:
      break;
    default:
      ROS_WARN("Unknow Turn Signal State!");
      turn_signal_is_ready_ = false;
      return;
    }

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    for (size_t i = start_id; i < end_id; i++) {
      base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, i, lateral);
      }
    break;
    
    base_waypoints_ = awp_plugins::smoothtransition(base_waypoints_, start_id, lateral_transition_rate_);
    ROS_INFO("Lane Change Command starting at %s", std::to_string(start_id).c_str());
    turn_signal_is_ready_ = false;
    mod_waypoints_pub_.publish(base_waypoints_);
  }
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPVelocityModify(DWPMod::RequestMsgs request, ros::Time stamp) {
  apsrc_msgs::CommandAccomplished ros_msg = {};
  ros_msg.msg_id = request.header.msg_id;
  ros_msg.request_id = request.header.request_id;
  ros_msg.request_stamp = stamp;
  ros_msg.request_accomplished = false; 
  
  if (request.header.request_id!=2){
    ROS_ERROR("Bad Message Handler");
    udp_report_pub_.publish(ros_msg);
  }

  std::unique_lock<std::mutex> status_lock(status_data_mtx_);
  if (received_base_waypoints_ and closest_waypoint_id_){
    int32_t start_id = closest_waypoint_id_;
    status_lock.unlock();

    uint32_t end_id = 0;
    int32_t last_id = base_waypoints_.waypoints.size() - 1;
    int32_t where_to_start = 0;
    if (request.velocityCmd.cmd.waypoint_id <= 0 and request.velocityCmd.cmd.number_of_waypoints > 0){
      where_to_start = std::min(last_id,
                                start_id - request.velocityCmd.cmd.waypoint_id);
      end_id = std::min(last_id,
                        where_to_start + request.velocityCmd.cmd.number_of_waypoints);
    } else if (request.velocityCmd.cmd.number_of_waypoints > 0) {
      where_to_start = std::min(last_id,
                                request.velocityCmd.cmd.waypoint_id);
      end_id = std::min(last_id,
                        where_to_start + request.velocityCmd.cmd.number_of_waypoints);
    } else if (request.velocityCmd.cmd.number_of_waypoints == -1) {
      where_to_start = std::min(last_id,
                                start_id - request.velocityCmd.cmd.waypoint_id);
      end_id = last_id;
    }
    double target_velocity = request.velocityCmd.cmd.magnitude > 0 ? request.velocityCmd.cmd.magnitude : 0;
    switch (request.velocityCmd.cmd.unit) {
      case 0: //m/s -> m/s
        break;
      case 1: //km/h -> m/s
        target_velocity /= 3.6;
        break;
      case 2://mph -> m/s
        target_velocity *= 0.44704;
        break;
    }

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    double min_velocity = 0;
    switch (request.velocityCmd.cmd.action) {
      case 0: //Modify
        for (uint i = where_to_start; i <= end_id; i++) {
          base_waypoints_.waypoints[i].twist.twist.linear.x =
                  std::min(max_speed_,
                            std::max(base_waypoints_.waypoints[i].twist.twist.linear.x + target_velocity,
                                    min_velocity));
        }
        ROS_INFO(
                "Following Waypoints velocity has been modified by %s [m/s]: %s->%s",
                std::to_string(target_velocity).c_str(),
                std::to_string(where_to_start).c_str(),
                std::to_string(end_id).c_str()
                );
        break;
      case 1: // Set
        for (uint i = where_to_start; i <= end_id; i++) {
          base_waypoints_.waypoints[i].twist.twist.linear.x =
                  std::min(max_speed_,
                            std::max(min_velocity, target_velocity)
                            );
        }
        ROS_INFO(
                "Following Waypoints velocity has been set to %s: %s->%s",
                std::to_string(target_velocity).c_str(),
                std::to_string(where_to_start).c_str(),
                std::to_string(end_id).c_str()
                );
        break;
    }
    if (request.velocityCmd.cmd.smoothingEn == 1){//auto
      if (where_to_start - 2 >= 0){
        double velocity_start =  base_waypoints_.waypoints[where_to_start-2].twist.twist.linear.x;
        double velocity_end =  base_waypoints_.waypoints[where_to_start+2].twist.twist.linear.x;
        for (int id = where_to_start - 2; id <= where_to_start + 2; id++){
          base_waypoints_.waypoints[id].twist.twist.linear.x =
                  (velocity_end - velocity_start) / 4 * (id - where_to_start - 2) + velocity_end;
        }
      }
      if (end_id + 2 <= base_waypoints_.waypoints.size()){
        double velocity_start =  base_waypoints_.waypoints[end_id-2].twist.twist.linear.x;
        double velocity_end =  base_waypoints_.waypoints[end_id+2].twist.twist.linear.x;
        for (uint id = end_id - 2; id <= end_id + 2; id++){
          base_waypoints_.waypoints[id].twist.twist.linear.x =
                  (velocity_end - velocity_start) / 4 * (id - end_id - 2) + velocity_end;
        }
      }
    }
    mod_waypoints_pub_.publish(base_waypoints_);
    ros_msg.request_accomplished = true;
  }

  ros_msg.header.stamp = ros::Time::now();
  udp_report_pub_.publish(ros_msg);
  return empty_udp_msg_;
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPPositionModify(DWPMod::RequestMsgs request, ros::Time stamp) {
  apsrc_msgs::CommandAccomplished ros_msg = {};
  ros_msg.msg_id = request.header.msg_id;
  ros_msg.request_id = request.header.request_id;
  ros_msg.request_stamp = stamp;
  ros_msg.request_accomplished = false;  

  std::unique_lock<std::mutex> status_lock(status_data_mtx_);
  if (received_base_waypoints_ and closest_waypoint_id_){
    int32_t start_id = closest_waypoint_id_;
    status_lock.unlock();

    int32_t end_id = 0;
    int32_t last_id = base_waypoints_.waypoints.size() - 1;
    int32_t where_to_start = 0;
    if (request.positionCmd.cmd.waypoint_id <= 0 and request.positionCmd.cmd.number_of_waypoints > 0){
      where_to_start = std::min(last_id, start_id - request.positionCmd.cmd.waypoint_id);
      end_id = std::min(last_id, where_to_start + request.positionCmd.cmd.number_of_waypoints);
    } else if (request.positionCmd.cmd.number_of_waypoints > 0) {
      where_to_start = std::min(last_id,
                                request.positionCmd.cmd.waypoint_id);
      end_id = std::min(last_id,
                        where_to_start + request.positionCmd.cmd.number_of_waypoints);
    } else if (request.positionCmd.cmd.number_of_waypoints == -1) {
      where_to_start = std::min(last_id,
                                start_id - request.positionCmd.cmd.waypoint_id);
      end_id = last_id;
    }
    double lateral = request.positionCmd.cmd.direction;
    switch (request.positionCmd.cmd.unit) {
      case 0:
        break;
      case 1:
        lateral *= 0.01;
        break;
      case 2:
        lateral *= 0.0254;
    }

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    switch (request.positionCmd.cmd.action) {
      case 0: //Modify
        for (int i = where_to_start +1; i < end_id; i++) {
          base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, i, lateral);
          }
        break;
    }
    if (request.positionCmd.cmd.smoothingEn == 1){
      base_waypoints_ = awp_plugins::smoothtransition(base_waypoints_, where_to_start+1, lateral_transition_rate_);
      base_waypoints_ = awp_plugins::smoothtransition(base_waypoints_, end_id, lateral_transition_rate_);
    }
    ROS_INFO("Following Waypoints has been shifted by %s: %s->%s", std::to_string(lateral).c_str(),
              std::to_string(where_to_start).c_str(), std::to_string(end_id).c_str());
    
    mod_waypoints_pub_.publish(base_waypoints_);
    ros_msg.request_accomplished = true;
  }

  ros_msg.header.stamp = ros::Time::now();

  udp_report_pub_.publish(ros_msg);
  return empty_udp_msg_;
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPVelocityVecModify(DWPMod::RequestMsgs request, ros::Time stamp) {
  apsrc_msgs::CommandAccomplished ros_msg = {};
  ros_msg.msg_id = request.header.msg_id;
  ros_msg.request_id = request.header.request_id;
  ros_msg.request_stamp = stamp;
  ros_msg.request_accomplished = false;

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t where_to_start;
  if (request.velocityVectorCmd.cmd.waypoint_id < 0){
    where_to_start = std::min(last_id, closest_waypoint_id_ - request.velocityVectorCmd.cmd.waypoint_id);
  }
  if(received_base_waypoints_ && closest_waypoint_id_){
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    size_t idx = 0;
    while(idx < request.velocityVectorCmd.cmd.num_of_waypoints && (where_to_start + idx) < last_id){
      base_waypoints_.waypoints[where_to_start + idx].twist.twist.linear.x = request.velocityVectorCmd.cmd.velocity_vector[idx];
      ++idx;
    }

    mod_waypoints_pub_.publish(base_waypoints_);
    ros_msg.request_accomplished = true;
    ROS_INFO("Following Waypoints velocity has been set using vector: %s->%s",
              std::to_string(where_to_start).c_str(),
              std::to_string(where_to_start+idx-1).c_str()
              );
  }

  ros_msg.header.stamp = ros::Time::now();
  udp_report_pub_.publish(ros_msg);
  return empty_udp_msg_;
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPPositionVecModify(DWPMod::RequestMsgs request, ros::Time stamp) {
  apsrc_msgs::CommandAccomplished ros_msg = {};
  ros_msg.msg_id = request.header.msg_id;
  ros_msg.request_id = request.header.request_id;
  ros_msg.request_stamp = stamp;
  ros_msg.request_accomplished = false;

  int32_t last_id = base_waypoints_.waypoints.size() - 1;
  int32_t where_to_start = request.positionVectorCmd.cmd.waypoint_id;
  if (request.positionVectorCmd.cmd.waypoint_id < 0){
    where_to_start = std::min(last_id, closest_waypoint_id_ - request.positionVectorCmd.cmd.waypoint_id);
  }

  if(received_base_waypoints_ && closest_waypoint_id_){
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    size_t idx = 0;
    while(idx < request.positionVectorCmd.cmd.num_of_waypoints && (where_to_start + idx) < last_id){
      base_waypoints_ = awp_plugins::shiftWaypoint(base_waypoints_, where_to_start + idx, request.positionVectorCmd.cmd.lat_shift_vector[idx]);
      ++idx;
    }

    mod_waypoints_pub_.publish(base_waypoints_);
    ROS_INFO("Following Waypoints velocity has been set using vector: %s->%s",
              std::to_string(where_to_start).c_str(),
              std::to_string(where_to_start+idx-1).c_str()
            );
    ros_msg.request_accomplished = true;
  }
    
  ros_msg.header.stamp = ros::Time::now();
  udp_report_pub_.publish(ros_msg);
  return empty_udp_msg_;
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPReset(DWPMod::RequestMsgs request, ros::Time stamp) {
  apsrc_msgs::CommandAccomplished ros_msg = {};
  ros_msg.msg_id = request.header.msg_id;
  ros_msg.request_id = request.header.request_id;
  ros_msg.request_stamp = stamp;
  ros_msg.request_accomplished = false;

  std::unique_lock<std::mutex> status_lock(status_data_mtx_);
  if (received_base_waypoints_ and closest_waypoint_id_){
    int32_t start_id  = closest_waypoint_id_;
    int32_t last_id = base_waypoints_.waypoints.size() - 1;
    status_lock.unlock();

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    for (int32_t i = start_id; i <= last_id; i++){
      base_waypoints_.waypoints[i] = original_waypoints_.waypoints[i];
    }
  
    mod_waypoints_pub_.publish(base_waypoints_);
    ROS_INFO("Waypoints have been reset!");
    ros_msg.request_accomplished = true;
  }
  
  ros_msg.header.stamp = ros::Time::now();
  udp_report_pub_.publish(ros_msg);
  return empty_udp_msg_;
}

}  // namespace apsrc_waypoint_replanner
PLUGINLIB_EXPORT_CLASS(apsrc_waypoint_replanner::ApsrcWaypointReplannerNl,
                       nodelet::Nodelet);
