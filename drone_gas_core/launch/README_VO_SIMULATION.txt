Visual odometry / SLAM demo (ROS 2 Jazzy + Gazebo Harmonic)

Prerequisites
-------------
After build: source install/setup.bash

Odometry tuning is applied via odom_args in drone_gas_core/launch/rtabmap_rgbd.launch.py.

If you still see matches but geometric inliers stay at 0, check rgbd_odometry was launched
with Vis/EstimationType=0 (3D→3D). RTAB‘s default EstimationType=1 is PnP; Vis/InlierDistance
mostly affects type 0, so tuning InlierDistance alone while type=1 is still PnP can look like
“nothing changes.”

(stock rtabmap.launch.py does not honor an arbitrary YAML params_file for these nodes.)

Mapping vs odometry order
-------------------------
RTAB-Map discards images until /rtabmap/odom is valid (quality > 0). There is nothing extra to
delay in launch: fixing rgbd_odometry fixes mapping automatically.

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

Optional 2D depth avoidance (demo only)
---------------------------------------
Rebuild and source workspace as usual, then launch with avoidance on:

  colcon build --packages-select drone_gas_core
  source install/setup.bash
  ros2 launch drone_gas_core full_system.launch.py enable_avoidance:=true enable_exploration:=false

Checks:
  ros2 topic echo /drone/cmd_vel           # Twist from simple_depth_avoidance_node → watchdog → bridge
  ros2 topic hz /drone/cmd_vel

Rules:
• Do NOT run visual_odometry_smoke_motion.py at the same time (two publishers on /drone/cmd_vel).
• Exploration is suppressed automatically whenever enable_avoidance:=true, even if
  enable_exploration:=true, so avoidance wins for a single-driver demo.

Tune (optional ros2 params on simple_depth_avoidance_node): safe_distance_m, forward_speed_m_s,
turn_speed_rad_s, roi_* fractions for front window.

Suggested demo storyline (gas + SLAM robot)
------------------------------------------
1) Simulation + RViz showing map growth from RGB-D SLAM (/rtabmap/odom, map/grid if enabled).
2) Gas sensor overlay (existing gas_sim + chemical_mapper) as a “risk map” or markers in RViz.
3) Optionally enable enable_avoidance:=true so the robot creeps forward and turns away from
   depth obstacles—still Nav2‑free and appropriate for fixed-height tabletop/hall demos.
