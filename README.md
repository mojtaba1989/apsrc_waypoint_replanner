# APSRC_waypoint_replanner

`apsrc_waypoint_replanner`
Acts as the UDP interface for executing commands coming from MABx.
The node updates the global path accordingly, and then sends back the status message to the MABx.

## Configuration

salammmm

See `config/params.yaml` for more details regarding configurable parameters such as network IPs and ports.

## Functional Overview

The apsrc_waypoint_replanner inserts itself at two places in the Autoware waypoint pipeline.
First, it reads in the global waypoints from the waypoints csv file (`/based/lane_waypoints_file` topic).
It then sets all waypoint velocities to the `max_speed` config parameter and sends these waypoints into the Autoware waypoint pipeline via the `/based/lane_waypoints_raw` topic.
At the end of the pipeline is the `/base_waypoints_envelope` topic which represents the fastest possible global velocity profile that still respects safe cornering lateral accelerations, linear accel/decel limits, and slowing for traffic lights.
The `/base_waypoints_envelope` topic is essentially a safety envelope that is used as the upper limit to constrain any unsafe requests from the MABx.
Second, ffter `/base_waypoints_envelope` is passed down to Autoware's local planning, other constraints like stop signs and stopping for objects are handled.
The velocity envelope represented by `/base_waypoints_envelope` can be tuned by adjusting parameters in `aw_platform/config/autonomy_params.yaml` such as `waypoint_replanner` parameters.
