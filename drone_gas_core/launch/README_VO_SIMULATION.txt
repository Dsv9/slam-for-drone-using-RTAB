Visual odometry / SLAM demo (ROS 2 Jazzy + Gazebo Harmonic)

Prerequisites
-------------
After build: source install/setup.bash

Odometry tuning is applied via odom_args in drone_gas_core/launch/rtabmap_rgbd.launch.py
(stock rtabmap.launch.py does not load params_file — see comment in config/rtabmap_params.yaml).

Bring-up (single cmd_vel publisher)
-----------------------------------
  ros2 launch drone_gas_core full_system.launch.py

Exploration publishes /drone/cmd_vel too — keep it off for VO tests:
  # enable_exploration defaults to false
  ros2 launch drone_gas_core full_system.launch.py enable_exploration:=false

Slow motion (+ optional yaw scan after odom_info inliers gate)
--------------------------------------------------------------
  ros2 run drone_gas_core visual_odometry_smoke_motion

Optional overrides example:
  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args \\
    -p linear_x:=0.03 -p scan_min_inliers:=15 -p scan_wait_timeout_sec:=30.0

Topic / TF checks
-----------------
  ros2 topic hz /rgbd_camera/image
  ros2 topic hz /rgbd_camera/depth_image
  ros2 topic hz /rgbd_camera/points
  ros2 topic hz /rtabmap/odom
  ros2 topic echo /rtabmap/odom_info --field inliers
  ros2 run tf2_ros tf2_echo base_link rgbd_camera

Success criteria (VO / SLAM)
----------------------------
• /rtabmap/odom_info inliers climbs and stays largely above ~10–20 once the drone creeps forward.
• /rtabmap/odom publishes; RTAB GUI / logs show periodic Odom quality > 0, not permanently 0.
• Log spam “Not enough inliers 0/20” should stop once rgbd_odometry sees Vis/MinInliers via odom_args (not the default 20).
• RViz: enable Path on /rtabmap/odom (or inspect map/cloud) once TF odom→base_link is live.
• RGB-D topics must remain: /rgbd_camera/image , /rgbd_camera/depth_image ,
  /rgbd_camera/camera_info , /rgbd_camera/points (do not rename).
