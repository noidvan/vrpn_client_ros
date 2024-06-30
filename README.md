# VICON VRPN Client ROS

*Modified by Ivan Lin*

Source: https://github.com/ros-drivers/vrpn_client_ros

Updated VICON bridge utilizing VRPN interface to publish velocity (twist) and acceleration measured by the VICON system.

## Dependencies
```bash
sudo apt-get install ros-$ROS_DISTRO-vrpn
```

VICON Tracker 3.3+

## Published Topics

 - `<tracker name>/odom` (nav_msgs/Odometry) (pose+twist)
 - `<tracker name>/pose` (geometry_msgs/PoseStamped)
 - `<tracker name>/twist` (geometry_msgs/TwistStamped)
 - `<tracker name>/accel` (geometry_msgs/AccelStamped)

## Launch
```bash
roslaunch vrpn_client_ros utias_vicon.launch
```

## Parameters
 - `update_frequency` (double, default: 100.0)
    - Frequency to process incoming updates from the VRPN server.
 - `refresh_tracker_frequency` (double, default: 0.0)
    - Frequency at which to auto-detect new Trackers (objects) from the VRPN server. Mutually exclusive with `trackers`.
 - `trackers` (list)
    - List of tracker names to expect from the VRPN server. Mutually exclusive with `refresh_tracker_frequency` > 0.0
 - `frame_id` (string, default: "world")
    - World frame id for the Trackers.
 - `twist_accel_in_body_frame` (bool, default: true)
    - Publish twist and acceleration in body frame instead of world frame.
 - `use_server_time` (bool, default: false)
    - Publish/broadcast using VRPN server's timestamp, or the local ROS time.
 - `broadcast_tf` (bool, default: true)
    - Broadcast transforms of Tracker pose on /tf.
 - `process_sensor_id` (bool, default: false)
    - In certain applications, there are multiple motion capture systems (sensors) tracking the same object under one network. Enabling this will allow result from different systems to be published as separate topics. Generally not applicable to our use case.