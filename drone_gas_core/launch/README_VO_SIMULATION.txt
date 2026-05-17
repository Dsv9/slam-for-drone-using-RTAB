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
