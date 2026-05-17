#!/usr/bin/env bash
# Quick RGB-D + VO sanity checks (run in a second terminal while full_system is up).
set -euo pipefail

echo "=== RGB-D topics ==="
ros2 topic list | grep rgbd || true

echo ""
echo "=== Rates (Ctrl+C each after ~5 s) ==="
echo "Expect ~8-10 Hz for image/depth/camera_info when Gazebo sim is running:"
echo "  ros2 topic hz /rgbd_camera/image"
echo "  ros2 topic hz /rgbd_camera/depth_image"
echo "  ros2 topic hz /rgbd_camera/camera_info"

echo ""
echo "=== Odometry (project uses /rtabmap/odom) ==="
timeout 8 ros2 topic hz /rtabmap/odom 2>/dev/null || echo "WARN: /rtabmap/odom not publishing yet"

echo ""
echo "=== One-shot odom + inliers ==="
timeout 3 ros2 topic echo /rtabmap/odom --once 2>/dev/null || echo "WARN: no /rtabmap/odom message"
timeout 3 ros2 topic echo /rtabmap/odom_info --field inliers --once 2>/dev/null || true

echo ""
echo "=== TF (expect odom -> base_link -> rgbd_camera) ==="
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -n 12 || echo "WARN: odom->base_link missing"
timeout 4 ros2 run tf2_ros tf2_echo base_link rgbd_camera 2>/dev/null | head -n 12 || echo "WARN: base_link->rgbd_camera missing"

echo ""
echo "=== Optional: slow VO motion (odom-only test; do NOT run with avoidance) ==="
echo "  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p linear_x:=0.02"
