Visual odometry / SLAM demo (ROS 2 Jazzy + Gazebo Harmonic)

Priority order (do not skip steps)
--------------------------------
1. Stable /rgbd_camera/* streams (~8-10 Hz).
2. rgbd_odometry inliers > 0 and /rtabmap/odom quality > 0.
3. RTAB-Map accepts images (no “no odometry is provided”).
4. Only then enable avoidance / exploration.

Prerequisites
-------------
  cd ~/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select drone_gas_core drone_gas_sim_bridge
  source install/setup.bash

Odometry tuning lives in launch/rtabmap_rgbd.launch.py (odom_args + rtabmap_args).
Key: Vis/EstimationType=0 (3D-3D). Type=1 (PnP) often gives “matches>0 but 0 inliers” in Gazebo.

Odom-only test (no auto motion)
-------------------------------
  ros2 launch drone_gas_core full_system.launch.py \
    enable_avoidance:=false \
    enable_exploration:=false \
    enable_rtabmap:=true \
    enable_rviz:=true \
    debug_odom:=true

With avoidance/exploration off, nothing publishes /drone/cmd_vel unless you run a separate node.

Optional slow creep to build parallax (after streams are stable; one cmd_vel publisher only):
  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p linear_x:=0.02

Full demo (after /rtabmap/odom works)
------------------------------------
  ros2 launch drone_gas_core full_system.launch.py \
    enable_avoidance:=true \
    enable_exploration:=false \
    debug_odom:=true

Debug script (second terminal)
------------------------------
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  bash $(ros2 pkg prefix drone_gas_core)/share/drone_gas_core/scripts/check_rgbd_odom.sh

Manual checks
-------------
  ros2 topic list | grep rgbd
  ros2 topic hz /rgbd_camera/image
  ros2 topic hz /rgbd_camera/depth_image
  ros2 topic hz /rgbd_camera/camera_info
  ros2 topic hz /rtabmap/odom
  ros2 topic echo /rtabmap/odom --once
  ros2 topic echo /rtabmap/odom_info --field inliers
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo base_link rgbd_camera

Expected results
----------------
• /rgbd_camera/image, depth_image, camera_info: ~8-10 Hz (not 0, not long gaps).
• /rtabmap/odom: continuous once VO locks (project uses namespace rtabmap, not bare /odom).
• /rtabmap/odom_info inliers: often > 5-10 while moving slowly; not stuck at 0 forever.
• rgbd_odometry: Odom quality > 0; no endless “Not enough inliers 0/8”.
• RTAB-Map: stops printing “no odometry is provided. Image is ignored.”
• RViz: map / cloud grows after valid odom.

If RGB/depth warnings at startup
--------------------------------
full_system delays RTAB-Map ~4 s so Gazebo + ros_gz_bridge can publish first.
Wait ~10 s after launch before judging hz.

TF / frames
-----------
  base_link -> rgbd_camera (static TF in full_system.launch.py)
  rgbd_camera -> simple_drone/base_link/rgbd_camera (alias for Gazebo headers)
  odom -> base_link (from rgbd_odometry when publish_tf_odom:=true)

Troubleshooting
---------------
• 0 inliers: wrong EstimationType, textureless view, or RGB-D not synced — check odom_args and world vo_calib_wall.
• Images ignored: /rtabmap/odom missing or quality=0 — fix VO before tuning mapping.
• Low Hz: Gazebo CPU; camera update_rate is 10 in model.sdf; close extra apps.
• Old world in sim: colcon build drone_gas_sim_bridge and restart Gazebo completely.

Optional 2D depth avoidance
-----------------------------
See full_system.launch.py enable_avoidance:=true (only after VO works).
