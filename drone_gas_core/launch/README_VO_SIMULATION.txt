Visual odometry / SLAM demo (ROS 2 Jazzy + Gazebo Harmonic)

Odometry topic
--------------
rgbd_odometry publishes nav_msgs/msg/Odometry on:

  /odom

(remap odom -> /odom in launch/rtabmap_rgbd.launch.py; namespace rtabmap is only for
 rtabmap/odom_info, map, etc.)

Priority order
--------------
1. Stable /rgbd_camera/* (~8-10 Hz).
2. /odom publishing with quality > 0 (check /rtabmap/odom_info inliers).
3. RTAB-Map accepts images (no “no odometry is provided”).
4. Then avoidance / exploration.

Prerequisites
-------------
  cd ~/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select drone_gas_core drone_gas_sim_bridge
  source install/setup.bash

Odom-only test (no auto cmd_vel)
--------------------------------
  ros2 launch drone_gas_core full_system.launch.py \
    enable_avoidance:=false \
    enable_exploration:=false \
    enable_rtabmap:=true \
    enable_rviz:=true \
    debug_odom:=true

Debug checks (second terminal)
------------------------------
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash

  ros2 topic list | grep -E "odom|rgbd|camera"
  ros2 topic info /odom
  ros2 topic hz /odom
  ros2 topic echo /odom --once
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo base_link rgbd_camera

  bash $(ros2 pkg prefix drone_gas_core)/share/drone_gas_core/scripts/check_rgbd_odom.sh

Expected success
----------------
• ros2 topic info /odom → nav_msgs/msg/Odometry
• ros2 topic hz /odom → stable rate when VO is healthy
• RViz “VO Odometry (/odom)” stops blinking red
• RTAB-Map stops “no odometry is provided”
• Map grows after slow motion

Manual Gazebo /cmd_vel test (bypasses avoidance — tests bridge + plugin)
---------------------------------------------------------------------------
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

If the drone does not move, the problem is NOT avoidance logic — check ros_gz_bridge
and simple_drone VelocityControl plugin on /cmd_vel.

Avoidance chain (when enable_avoidance:=true):
  simple_depth_avoidance_node -> /drone/cmd_vel
  cmd_vel_watchdog_node -> /drone/cmd_vel_safe
  gazebo_controller_bridge_node -> /cmd_vel
  ros_gz parameter_bridge -> Gazebo

Optional VO motion (one cmd_vel publisher only)
-----------------------------------------------
  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p linear_x:=0.02

Full demo (after /odom works)
-----------------------------
  ros2 launch drone_gas_core full_system.launch.py \
    enable_avoidance:=true \
    enable_exploration:=false \
    debug_odom:=true

TF
--
  map -> odom -> base_link -> rgbd_camera
  (+ alias rgbd_camera -> simple_drone/base_link/rgbd_camera)

Troubleshooting
---------------
• /odom missing but /rtabmap/odom exists: rebuild drone_gas_core; odom_topic must be /odom.
• 0 inliers: see odom_args in rtabmap_rgbd.launch.py (Vis/EstimationType=0).
• Startup “depth not published”: wait ~10 s; RTAB starts ~4 s after Gazebo.
