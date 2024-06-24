# APSRC_waypoint_replanner

# Packet Definitions

This repository contains a C++ header file defining packet structures and classes for a communication protocol used in the DWPMod system.

## Overview

The `packet_definitions.h` file defines various packet structures and classes for handling different types of commands and data in the DWPMod system.

## Structure Details

### `smoothing_ctrl_t`

- `beginning`: uint8_t. Specifies the beginning type (1 for fix steps, 2 for fix number).
- `ending`: uint8_t. Specifies the ending type (1 for fix steps, 2 for fix number).
- `beginning_smoothing_extra`: float. Extra smoothing parameter for the beginning.
- `ending__smoothing_extra`: float. Extra smoothing parameter for the ending.

### `velocityCMD_t`

- `waypoint_id`: int32_t. ID of the waypoint.
- `number_of_waypoints`: int32_t. Number of waypoints.
- `action`: uint8_t. Action type (0 for modify, 1 for set).
- `magnitude`: float. Velocity magnitude.
- `unit`: uint8_t. Velocity unit (0 for m/s, 1 for km/h, 2 for mph).
- `smoothingEn`: uint8_t. Smoothing enable status.
- `smoothingCtrl`: struct `smoothing_ctrl_t`. Smoothing control parameters.

### `positionCMD_t`

- `waypoint_id`: int32_t. ID of the waypoint.
- `number_of_waypoints`: int32_t. Number of waypoints.
- `action`: uint8_t. Action type (0 for modify, 1 for add, 2 for remove).
- `direction`: float. Lateral direction.
- `unit`: uint8_t. Unit of direction (0 for m, 1 for cm, 2 for inch).
- `smoothingEn`: uint8_t. Smoothing enable status.
- `smoothingCtrl`: struct `smoothing_ctrl_t`. Smoothing control parameters.

### `velocityProfileCMD_t`

- `waypoint_id`: int32_t. ID of the waypoint.
- `num_of_waypoints`: uint8_t. Number of waypoints.
- `velocity_vector`: float[200]. Array of velocity vectors.

### `positionProfileCMD_t`

- `waypoint_id`: int32_t. ID of the waypoint.
- `num_of_waypoints`: uint8_t. Number of waypoints.
- `lat_shift_vector`: float[200]. Array of lateral shift vectors.

### `header`

- `msg_id`: uint8_t. Message ID.
- `request_id`: uint8_t. Request ID.
- `time_stamp`: int32_t[2]. Time stamp.
- `data_size_byte`: uint32_t. Size of the data in bytes.
- `data_info`: uint8_t[10]. Information about the data.

### Classes for Unpacking

- `velocityCMD`: Class for unpacking velocity command packets.
- `positionCMD`: Class for unpacking position command packets.
- `veocityVectorCMD`: Class for unpacking velocity vector command packets.
- `positionVectorCMD`: Class for unpacking position vector command packets.
- `RequestMsgs`: Class for unpacking request message packets.

### Rquest Message Structure:`RequestMsgs`
Total length: 1024 bytes
1. header: `header` 24 bytes
2.  body: only one of following commands
    - `velocityCMD`: 15 bytes + 981 NULL bytes
    - `positionCMD`: 15 bytes + 981 NULL bytes
    - `veocityVectorCMD`: 805 bytes + 191 NULL bytes
    - `positionVectorCMD`: 805 bytes + 191 NULL bytes
- `CRC`: 4 bytes

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

