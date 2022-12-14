#include <string>
#include <vector>

#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_nodelet.hpp"
#include "apsrc_waypoint_replanner/packet_definitions.hpp"

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
  mod_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcWaypointReplannerNl::velocityCallback, this);
  vehicle_status_sub_ = nh_.subscribe("vehicle_status", 1, &ApsrcWaypointReplannerNl::vehicleStatusCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &ApsrcWaypointReplannerNl::closestWaypointCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &ApsrcWaypointReplannerNl::baseWaypointsCallback, this);

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
  pnh_.param("max_speed", max_speed_, 100.0);
  pnh_.param("time_out", time_out_, 0.5);

  const ros::Duration OUTDATED_DATA_TIMEOUT(time_out_);

  ROS_INFO("Parameters Loaded");
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  DVPMod::RequestMsgs requestMsg;
  if (!requestMsg.unpack(received_payload))
  {
    ROS_ERROR("Checksum doesn't match! dropping packet");
  }
  else
  {
    std::vector<uint8_t> udp_msg;
    switch (requestMsg.request_id) {
      case 1: // request for waypoints from spectra
        ROS_INFO("Received request: sharing waypoint");
        udp_msg = ApsrcWaypointReplannerNl::UDPGlobalPathShare(current_waypoints_);
        break;

      case 2: // request for velocity modification
        ROS_INFO("Received request: velocity modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPVelocityModify(requestMsg);
        break;
    }
    return udp_msg;
  }
}

void ApsrcWaypointReplannerNl::generateGlobalPath(DVPMod::VelocityProfile velocity_profile)
{
  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);

  if (received_base_waypoints_)
  {
    autoware_msgs::Lane modified_waypoints = base_waypoints_;

    uint num_waypoints = modified_waypoints.waypoints.size();

    if (num_waypoints != base_waypoints_.waypoints.size())
    {
      ROS_ERROR("Waypoints don't match! unable to modify velocities");
    }
    else
    {
      // Now modify based on UDP packet profile, never exceeding envelope
      if (velocity_profile.status == 1)
      {
        for (uint i = 0; i < 50; ++i)
        {
          uint wp_id = i + velocity_profile.first_global_waypoint_id;
          if (wp_id < num_waypoints)
          {
            double profile_vel = static_cast<double>(velocity_profile.target_velocities[i]) / 1000.0;

            modified_waypoints.waypoints[wp_id].twist.twist.linear.x =
                std::min(profile_vel, max_speed_);
          }
        }
        std::unique_lock<std::mutex> status_lock(status_data_mtx_);
        velocity_profile_enabled_ = true;
      }
      else
      {
        std::unique_lock<std::mutex> status_lock(status_data_mtx_);
        velocity_profile_enabled_ = false;
      }

      std::unique_lock<std::mutex> status_lock(status_data_mtx_);
      target_velocity_ = static_cast<uint16_t>(
          std::round(std::abs(modified_waypoints.waypoints[closest_waypoint_id_].twist.twist.linear.x) * 1000.0));
    }

    mod_waypoints_pub_.publish(modified_waypoints);
    base_waypoints_ = modified_waypoints;
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "Waiting for waypoints");
  }
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
        std::bind(&ApsrcWaypointReplannerNl::handleServerResponse, this, std::placeholders::_1));

    return true;
  }
}

void ApsrcWaypointReplannerNl::baseWaypointsCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints)
{
  if (!received_base_waypoints_)
  {
    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    base_waypoints_ = *base_waypoints;
    autoware_msgs::Lane mod_waypoints = *base_waypoints;
//    for (auto& waypoint : max_waypoints.waypoints)
//    {
//      waypoint.twist.twist.linear.x = max_speed_;
//    }
//
    mod_waypoints_pub_.publish(base_waypoints_);
    current_waypoints_ = mod_waypoints;
    received_base_waypoints_ = true;
  }
}

void ApsrcWaypointReplannerNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x) * 1000.0));
  current_velocity_rcvd_time_ = ros::Time::now();
}

void ApsrcWaypointReplannerNl::vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  dbw_engaged_ = (vehicle_status->drivemode == autoware_msgs::VehicleStatus::MODE_AUTO ||
                  vehicle_status->steeringmode == autoware_msgs::VehicleStatus::MODE_AUTO);
  dbw_engaged_rcvd_time_ = ros::Time::now();
}

void ApsrcWaypointReplannerNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
  closest_waypoint_id_rcvd_time_ = ros::Time::now();
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPGlobalPathShare(autoware_msgs::Lane_<std::allocator<void>> waypoints)
{
  std::unique_lock<std::mutex> status_lock(status_data_mtx_);
  if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
    int32_t start_id = closest_waypoint_id_;
    status_lock.unlock();

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    autoware_msgs::Lane temp_waypoints = waypoints;

    DVPMod::WaypointsArray udp_msg;
    udp_msg.msg_id = msg_id_;
    udp_msg.type = 1;
    udp_msg.num_waypoints = (temp_waypoints.waypoints.size() - start_id < 100) ? temp_waypoints.waypoints.size() -
                                                                                 start_id : 100;
    udp_msg.first_global_waypoint_id = start_id;
    udp_msg.crc = 0;
    for (uint i = 0; i < udp_msg.num_waypoints; i++) {
      uint wp_id = i + start_id;
      udp_msg.waypoints_array[i].waypoint_id = temp_waypoints.waypoints[wp_id].gid;
      udp_msg.waypoints_array[i].x = temp_waypoints.waypoints[wp_id].pose.pose.position.x;
      udp_msg.waypoints_array[i].y = temp_waypoints.waypoints[wp_id].pose.pose.position.y;
      udp_msg.waypoints_array[i].z = temp_waypoints.waypoints[wp_id].pose.pose.position.z;
      udp_msg.waypoints_array[i].yaw = tf::getYaw(temp_waypoints.waypoints[wp_id].pose.pose.orientation);
      udp_msg.waypoints_array[i].velocity = temp_waypoints.waypoints[wp_id].twist.twist.linear.x;
      udp_msg.waypoints_array[i].change_flag = temp_waypoints.waypoints[wp_id].change_flag;
    }
    ROS_INFO("%s Following Waypoints has shared, starting %s", std::to_string(udp_msg.num_waypoints).c_str(),
             std::to_string(udp_msg.first_global_waypoint_id).c_str());
    return udp_msg.pack();
  }
  else
  {
    status_lock.unlock();
    DVPMod::WaypointsArray udp_msg;
    ROS_WARN("No Waypoints to Share");
    return udp_msg.pack();
  }
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPVelocityModify(DVPMod::RequestMsgs request) {
  DVPMod::StatusReply reply={};
  if (request.request_id!=2){
    ROS_ERROR("Bad Message Handler");
  } else{
    std::unique_lock<std::mutex> status_lock(status_data_mtx_);
    if (received_base_waypoints_ and closest_waypoint_id_){
      int32_t start_id = closest_waypoint_id_;
      status_lock.unlock();

      uint32_t end_id =
              request.reserved.waypoint_id + request.reserved.number_of_waypoints < base_waypoints_.waypoints.size() ?
              request.reserved.waypoint_id + request.reserved.number_of_waypoints - 1: base_waypoints_.waypoints.size() - 1;
      uint32_t where_to_start = request.reserved.waypoint_id > start_id ? request.reserved.waypoint_id:start_id;

      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      switch (request.reserved.action) {
        case 0: //Modify
          for (int i = where_to_start; i <= end_id; i++) {
            base_waypoints_.waypoints[i].twist.twist.linear.x =
                    std::min(max_speed_, base_waypoints_.waypoints[i].twist.twist.linear.x + request.reserved.magnitude);
          }
          ROS_INFO("Following Waypoints has been modified by %s: %s->%s", std::to_string(request.reserved.magnitude).c_str(),
                   std::to_string(where_to_start).c_str(), std::to_string(end_id).c_str());
          break;
        case 1: // Set
          for (int i = where_to_start; i <= end_id; i++) {
            base_waypoints_.waypoints[i].twist.twist.linear.x = std::min(max_speed_, static_cast<double>(request.reserved.magnitude));
          }
          ROS_INFO("Following Waypoints has been set by %s: %s->%s", std::to_string(request.reserved.magnitude).c_str(),
                   std::to_string(where_to_start).c_str(), std::to_string(end_id).c_str());
          break;
      }
      mod_waypoints_pub_.publish(base_waypoints_);

      std::unique_lock<std::mutex> lock(status_data_mtx_);
      reply.data_valid = true;
      reply.closest_global_waypoint_id = closest_waypoint_id_;
      reply.current_velocity = current_velocity_;
    }
  }
  return reply.pack();
}
}  // namespace apsrc_waypoint_replanner

PLUGINLIB_EXPORT_CLASS(apsrc_waypoint_replanner::ApsrcWaypointReplannerNl, nodelet::Nodelet);
