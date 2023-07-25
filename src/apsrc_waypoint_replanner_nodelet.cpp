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
  mod_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1,
                                        &ApsrcWaypointReplannerNl::velocityCallback, this);
  vehicle_status_sub_ = nh_.subscribe("vehicle_status", 1,
                                      &ApsrcWaypointReplannerNl::vehicleStatusCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1,
                                        &ApsrcWaypointReplannerNl::closestWaypointCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1,
                                      &ApsrcWaypointReplannerNl::baseWaypointsCallback, this);

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
  pnh_.param<std::string>("/waypoint_replanner/server_ip", server_ip_, "127.0.0.1");
  pnh_.param("/waypoint_replanner/server_port", server_port_, 1551);
  pnh_.param("/waypoint_replanner/max_speed", max_speed_, 100.0);
  pnh_.param("/waypoint_replanner/max_yaw_rate", max_yaw_rate_, 1.0);
  pnh_.param("/waypoint_replanner/time_out", time_out_, 0.5);
  pnh_.param("/waypoint_replanner/time_gap", time_gap_, 0.5);
  pnh_.param("/waypoint_replanner/lateral_transition_duration", lateral_transition_duration_, 2.0);


  const ros::Duration OUTDATED_DATA_TIMEOUT(time_out_);

  ROS_INFO("Parameters Loaded");
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  std::vector<uint8_t> udp_msg = {};
  DVPMod::RequestMsgs requestMsg;
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
    switch (requestMsg.header.request_id) {
      case 1: // request for waypoints from spectra
        ROS_INFO("Received request: sharing waypoint");
        udp_msg = ApsrcWaypointReplannerNl::UDPGlobalPathShare(requestMsg);
        last_msg_time_ = ros::Time::now();
        break;

      case 2: // request for velocity modification
        ROS_INFO("Received request: velocity modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPVelocityModify(requestMsg);
        last_msg_time_ = ros::Time::now();
        break;
      case 3: // request for position shift
        ROS_INFO("Received request: position modification");
        udp_msg = ApsrcWaypointReplannerNl::UDPPositionModify(requestMsg);
        last_msg_time_ = ros::Time::now();
        break;
      case 4: // request for status sharing
        ROS_INFO("Received request: sharing status");
        udp_msg = ApsrcWaypointReplannerNl::UDPStatusShare(requestMsg);
        last_msg_time_ = ros::Time::now();
        break;
      case 255: // request for reset
        ROS_INFO("Received request: reset");
        udp_msg = ApsrcWaypointReplannerNl::UDPReset(requestMsg);
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

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPGlobalPathShare(DVPMod::RequestMsgs request)
{
  double start_time = ros::Time::now().toSec();
  std::unique_lock<std::mutex> status_lock(status_data_mtx_);
  DVPMod::ServiceReply udp_msg;
  udp_msg.header.msg_id = msg_id_; ++msg_id_;// need to be more meaningful
  udp_msg.header.request_id = 1;
  udp_msg.header.time_stamp[0] = ros::Time::now().sec;
  udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
  udp_msg.crc = 0;
  udp_msg.service_msg_id = request.header.msg_id;
  udp_msg.service_request_id = request.header.request_id;
  if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
    int32_t start_id = closest_waypoint_id_;
    status_lock.unlock();

    std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
    autoware_msgs::Lane temp_waypoints = base_waypoints_;

    udp_msg.header.data_size_byte = 2800;
    udp_msg.waypoints_array_msg.num_waypoints = (temp_waypoints.waypoints.size() - start_id < 100) ?
            temp_waypoints.waypoints.size() - start_id : 100;
    udp_msg.waypoints_array_msg.first_global_waypoint_id = start_id;
    for (uint i = 0; i < udp_msg.waypoints_array_msg.num_waypoints; i++) {
      uint wp_id = i + start_id;
      udp_msg.waypoints_array_msg.waypoints_array[i].waypoint_id =
              temp_waypoints.waypoints[wp_id].gid;
      udp_msg.waypoints_array_msg.waypoints_array[i].x =
              temp_waypoints.waypoints[wp_id].pose.pose.position.x;
      udp_msg.waypoints_array_msg.waypoints_array[i].y =
              temp_waypoints.waypoints[wp_id].pose.pose.position.y;
      udp_msg.waypoints_array_msg.waypoints_array[i].z =
              temp_waypoints.waypoints[wp_id].pose.pose.position.z;
      udp_msg.waypoints_array_msg.waypoints_array[i].yaw =
              tf::getYaw(temp_waypoints.waypoints[wp_id].pose.pose.orientation);
      udp_msg.waypoints_array_msg.waypoints_array[i].velocity =
              temp_waypoints.waypoints[wp_id].twist.twist.linear.x;
      udp_msg.waypoints_array_msg.waypoints_array[i].change_flag =
              temp_waypoints.waypoints[wp_id].change_flag;
    }
    ROS_INFO(
            "%s Following Waypoints has been shared, starting %s",
            std::to_string(udp_msg.waypoints_array_msg.num_waypoints).c_str(),
            std::to_string(udp_msg.waypoints_array_msg.first_global_waypoint_id).c_str()
            );
    udp_msg.service_accomplished = true;
  }
  else
  {
    status_lock.unlock();
    ROS_WARN("No Waypoints to Share");
    udp_msg.service_accomplished = true;
  }
  double end_time = ros::Time::now().toSec();
  udp_msg.processing_time = static_cast<float>((end_time - start_time) * 1000000);
  udp_msg.header.time_stamp[0] = ros::Time::now().sec;
  udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
  return udp_msg.pack();
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPVelocityModify(DVPMod::RequestMsgs request) {
  double start_time = ros::Time::now().toSec();
  DVPMod::ServiceReply udp_msg={};
  udp_msg.header.msg_id = msg_id_; ++msg_id_;// need to be more meaningful
  udp_msg.header.request_id = 2;
  udp_msg.header.data_size_byte = 0;
  udp_msg.crc = 0;
  udp_msg.service_msg_id = request.header.msg_id;
  udp_msg.service_request_id = request.header.request_id;
  if (request.header.request_id!=2){
    ROS_ERROR("Bad Message Handler");
    udp_msg.service_accomplished = false;
  } else{
    std::unique_lock<std::mutex> status_lock(status_data_mtx_);
    if (received_base_waypoints_ and closest_waypoint_id_){
      int32_t start_id = closest_waypoint_id_;
      status_lock.unlock();

      int32_t end_id = 0;
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t where_to_start = 0;
      if (request.positionCmd.cmd.waypoint_id <= 0 and request.positionCmd.cmd.number_of_waypoints > 0){
        where_to_start = std::min(last_id,
                                  start_id - request.positionCmd.cmd.waypoint_id);
        end_id = std::min(last_id,
                          where_to_start + request.positionCmd.cmd.number_of_waypoints);
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
          for (int i = where_to_start; i <= end_id; i++) {
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
          for (int i = where_to_start; i <= end_id; i++) {
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
          for (int id = end_id - 2; id <= end_id + 2; id++){
            base_waypoints_.waypoints[id].twist.twist.linear.x =
                    (velocity_end - velocity_start) / 4 * (id - end_id - 2) + velocity_end;
          }
        }
      }
      mod_waypoints_pub_.publish(base_waypoints_);
      udp_msg.service_accomplished = true;
    }
  }
  double end_time = ros::Time::now().toSec();
  udp_msg.processing_time = static_cast<float>((end_time - start_time)*1000000);
  udp_msg.header.time_stamp[0] = ros::Time::now().sec;
  udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
  return udp_msg.pack();
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPPositionModify(DVPMod::RequestMsgs request) {
  double start_time = ros::Time::now().toSec();
  DVPMod::ServiceReply udp_msg={};
  udp_msg.header.msg_id = msg_id_; ++msg_id_;// need to be more meaningful
  udp_msg.header.request_id = 3;
  udp_msg.header.data_size_byte = 0;
  udp_msg.crc = 0;
  udp_msg.service_msg_id = request.header.msg_id;
  udp_msg.service_request_id = request.header.request_id;
  if (request.header.request_id!=3) {
    ROS_ERROR("Bad Message Handler");
    udp_msg.service_accomplished = false;
  } else {
    std::unique_lock<std::mutex> status_lock(status_data_mtx_);
    if (received_base_waypoints_ and closest_waypoint_id_){
      int32_t start_id = closest_waypoint_id_;
      status_lock.unlock();

      int32_t end_id = 0;
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      int32_t where_to_start = 0;
      if (request.positionCmd.cmd.waypoint_id <= 0 and request.positionCmd.cmd.number_of_waypoints > 0){
        where_to_start = std::min(last_id,
                                  start_id - request.positionCmd.cmd.waypoint_id);
        end_id = std::min(last_id,
                          where_to_start + request.positionCmd.cmd.number_of_waypoints);
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
            base_waypoints_ = dvp_plugins::shiftWaypoint(base_waypoints_, i, lateral);
            }
          break;
      }
      if (request.positionCmd.cmd.smoothingEn == 1){
        base_waypoints_ = dvp_plugins::smoothtransition(base_waypoints_, where_to_start+1, lateral_transition_rate_);
        base_waypoints_ = dvp_plugins::smoothtransition(base_waypoints_, end_id, lateral_transition_rate_);
      }
      ROS_INFO("Following Waypoints has been shifted by %s: %s->%s", std::to_string(lateral).c_str(),
               std::to_string(where_to_start).c_str(), std::to_string(end_id).c_str());
      mod_waypoints_pub_.publish(base_waypoints_);
      udp_msg.service_accomplished = true;
    }
  }
  double end_time = ros::Time::now().toSec();
  udp_msg.processing_time = static_cast<float>((end_time - start_time) * 1000000);
  udp_msg.header.time_stamp[0] = ros::Time::now().sec;
  udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
  return udp_msg.pack();
}

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPStatusShare(DVPMod::RequestMsgs request) {
  DVPMod::ServiceReply udp_msg = {};
  udp_msg.header.msg_id = msg_id_;
  ++msg_id_;// need to be more meaningful
  udp_msg.header.request_id = 4;
  udp_msg.header.data_size_byte = 0;
  udp_msg.crc = 0;
  udp_msg.service_msg_id = request.header.msg_id;
  udp_msg.service_request_id = request.header.request_id;
  if (request.header.request_id != 4) {
    ROS_ERROR("Bad Message Handler");
    udp_msg.service_accomplished = false;
  } else {
    std::unique_lock<std::mutex> status_lock(status_data_mtx_);
    if (received_base_waypoints_ and closest_waypoint_id_) {
      udp_msg.status_msg.current_velocity = current_velocity_;
      udp_msg.status_msg.target_global_velocity = target_velocity_;
      udp_msg.status_msg.closest_global_waypoint_id = closest_waypoint_id_;
      udp_msg.service_accomplished = true;
    }
    udp_msg.header.time_stamp[0] = ros::Time::now().sec;
    udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
    ROS_INFO("Status has been shared");
    return udp_msg.pack();
  }
};

std::vector<uint8_t> ApsrcWaypointReplannerNl::UDPReset(DVPMod::RequestMsgs request) {
  double start_time = ros::Time::now().toSec();
  DVPMod::ServiceReply udp_msg = {};
  udp_msg.header.msg_id = msg_id_;
  ++msg_id_;// need to be more meaningful
  udp_msg.header.request_id = 255;
  udp_msg.header.data_size_byte = 0;
  udp_msg.crc = 0;
  udp_msg.service_msg_id = request.header.msg_id;
  udp_msg.service_request_id = request.header.request_id;
  if (request.header.request_id != 255) {
    ROS_ERROR("Bad Message Handler");
    udp_msg.service_accomplished = false;
  } else {
    std::unique_lock<std::mutex> status_lock(status_data_mtx_);
    if (received_base_waypoints_ and closest_waypoint_id_){
      int32_t start_id  = closest_waypoint_id_;
      int32_t last_id = base_waypoints_.waypoints.size() - 1;
      status_lock.unlock();

      std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
      for (int32_t i = start_id; i <= last_id; i++){
        base_waypoints_.waypoints[i] = original_waypoints_.waypoints[i];
      }
  }
  mod_waypoints_pub_.publish(base_waypoints_);
  udp_msg.service_accomplished = true;
  ROS_INFO("Waypoints have been reset!");
  double end_time = ros::Time::now().toSec();
  udp_msg.processing_time = static_cast<float>((end_time - start_time)*1000000);
  udp_msg.header.time_stamp[0] = ros::Time::now().sec;
  udp_msg.header.time_stamp[1] = ros::Time::now().nsec;
  return udp_msg.pack();
  }

}
}  // namespace apsrc_waypoint_replanner

PLUGINLIB_EXPORT_CLASS(apsrc_waypoint_replanner::ApsrcWaypointReplannerNl,
                       nodelet::Nodelet);
