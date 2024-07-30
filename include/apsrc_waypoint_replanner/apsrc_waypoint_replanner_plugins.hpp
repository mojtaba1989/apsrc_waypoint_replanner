#ifndef AWPRP_PLUGINS_H
#define AWPRP_PLUGINS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <mutex>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include "apsrc_waypoint_replanner/packet_definitions.hpp"
#include "apsrc_waypoint_replanner/apsrc_waypoint_replanner_nodelet.hpp"

namespace awp_plugins
{
autoware_msgs::Lane smoothtransition(autoware_msgs::Lane waypoints, uint32_t id, double lateral_transition_rate);
autoware_msgs::Lane shiftWaypoint(autoware_msgs::Lane waypoints, uint32_t id, double shift);
bool updateQuaternion(autoware_msgs::Lane waypoints, uint32_t id);

autoware_msgs::Lane smoothtransition(autoware_msgs::Lane waypoints, uint32_t id, double lateral_transition_rate)
{
  if (id >1 and id < waypoints.waypoints.size()-1){
    bool end_process = false;
    double area = (waypoints.waypoints[id-2].pose.pose.position.x - waypoints.waypoints[id-1].pose.pose.position.x) 
    * (waypoints.waypoints[id].pose.pose.position.y - waypoints.waypoints[id-1].pose.pose.position.y)
      - (waypoints.waypoints[id].pose.pose.position.x - waypoints.waypoints[id-1].pose.pose.position.x)
      * (waypoints.waypoints[id-2].pose.pose.position.y - waypoints.waypoints[id-1].pose.pose.position.y);
    // double yaw0 = tf::getYaw(waypoints.waypoints[id-1].pose.pose.orientation);
    // double yaw1 = tf::getYaw(waypoints.waypoints[id].pose.pose.orientation);
    double sign = area > 0 ? 1:-1;
    while (not end_process){
      double yaw0 = tf::getYaw(waypoints.waypoints[id-1].pose.pose.orientation);
      // double yaw1 = tf::getYaw(waypoints.waypoints[id].pose.pose.orientation);
      double A = std::tan(yaw0);
      double C = waypoints.waypoints[id-1].pose.pose.position.y -
                  waypoints.waypoints[id-1].pose.pose.position.x * A;
      double lateral_diff = std::abs(A * waypoints.waypoints[id].pose.pose.position.x -
                                    waypoints.waypoints[id].pose.pose.position.y + C) * std::abs(std::cos(yaw0));
      double lateral_allowed = lateral_transition_rate / waypoints.waypoints[id-1].twist.twist.linear.x;
      if (lateral_diff <= lateral_allowed){
        end_process = true;
      } else {
        double dl = -sign * (lateral_diff - lateral_allowed) ;
        waypoints = shiftWaypoint(waypoints, id, dl);
        id++;
        if (id == waypoints.waypoints.size()-1){
          end_process = true;
        }
      }
    }
  }
  return waypoints;
}

autoware_msgs::Lane shiftWaypoint(autoware_msgs::Lane waypoints, uint32_t id, double shift)
{
  float yaw = tf::getYaw(waypoints.waypoints[id].pose.pose.orientation);
  float dx = shift * sin(yaw);
  float dy = -shift * cos(yaw);
  waypoints.waypoints[id].pose.pose.position.x += dx;
  waypoints.waypoints[id].pose.pose.position.y += dy;
  return waypoints;
}

bool updateQuaternion(autoware_msgs::Lane waypoints, uint32_t id)
{
  if (id < waypoints.waypoints.size()-1){
    double x1 = waypoints.waypoints[id].pose.pose.position.x;
    double y1 = waypoints.waypoints[id].pose.pose.position.y;
    double x2 = waypoints.waypoints[id+1].pose.pose.position.x;
    double y2 = waypoints.waypoints[id+1].pose.pose.position.y;

    double dx = x2 - x1;
    double dy = y2 - y1;

    double angle = std::atan2(dy, dx);
    waypoints.waypoints[id].pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    return true;
  }
  return false;
}
};
#endif //AWPRP_PLUGINS_H

  