# APSRC_waypoint_replanner V2.0

This ROS package implements a waypoint replanner for the APSRC project. The waypoint replanner is responsible for generating a sequence of waypoints that guide the vehicle along a desired path. The replanner is designed to be modular and flexible, allowing for easy integration with other components of the APSRC system.

Changes in V2.0:
- All udp communication is removed. apsrc_udp_rosbridge is responsible for udp communication and converting MABx commands into ros messages.
- Added a driver extension node that processes driver input reports to generate and publish corresponding driver input commands.


# Waypoint Replanner Driver Extension Node (Pacifica ONLY)

This ROS package implements a waypoint replanner driver extension node that processes driver input reports to generate and publish corresponding driver input commands.

## Functionality Overview

The node subscribes to driver input reports, interprets various driver inputs such as turn signals, cruise control buttons, and lane change requests, and publishes appropriate commands for the waypoint replanner.

### Topics

#### Subscribed Topics

- **`/vehicle/driver_input_report`** (type: `raptor_dbw_msgs::DriverInputReport`)
  - Receives driver input reports which include turn signals and various cruise control buttons.

#### Published Topics

- **`waypoint_replanner/driver_command`** (type: `apsrc_msgs::DriverInputCommand`)
  - Publishes interpreted driver commands such as lane changes, speed adjustments, and waypoint resets.

<img src="https://github.com/mojtaba1989/apsrc_waypoint_replanner/blob/master//SteeringWheel.jpg" alt="Disks" width="600"/>

### Driver Commands

- **Left Lane Change**: Triggered by the left turn signal.
- **Right Lane Change**: Triggered by the right turn signal.
- **Speed Up**: Triggered by the cruise control accelerate button.
- **Speed Down**: Triggered by the cruise control decelerate button.
- **Increase Time Gap**: Triggered by the adaptive cruise increase distance button.
- **Decrease Time Gap**: Triggered by the adaptive cruise decrease distance button.
- **Reset Waypoints**: Triggered by the cruise resume button.

This functionality allows for effective and responsive control of the waypoint replanner based on driver inputs, enhancing the vehicle's adaptive capabilities.

