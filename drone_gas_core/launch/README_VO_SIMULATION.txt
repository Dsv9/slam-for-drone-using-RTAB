Visual odometry / SLAM demo checks (ROS 2 Jazzy, after: source install/setup.bash)

1) Launch (exploration OFF by default — only smoke motion should drive /drone/cmd_vel):
   ros2 launch drone_gas_core full_system.launch.py

   Optional: enable wandering explorer (not recommended while tuning VO):
   ros2 launch drone_gas_core full_system.launch.py enable_exploration:=true

2) Slow open-loop motion (default parameters are conservative):
   ros2 run drone_gas_core visual_odometry_smoke_motion

   Optional yaw “scan” after forward motion:
   ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p scan_enabled:=true

3) Topic rates (sim playing):
   ros2 topic hz /rgbd_camera/image
   ros2 topic hz /rgbd_camera/depth_image
   ros2 topic hz /rgbd_camera/points
   ros2 topic hz /rtabmap/odom

4) TF sanity (should print a valid transform):
   ros2 run tf2_ros tf2_echo base_link rgbd_camera

5) If RGB-D messages still show the long Gazebo frame in headers, verify
   drone_gas_sim_bridge/config/gz_ros_bridge.yaml uses frame_id: rgbd_camera on each
   bridged sensor entry (matches static TF in full_system.launch.py).
