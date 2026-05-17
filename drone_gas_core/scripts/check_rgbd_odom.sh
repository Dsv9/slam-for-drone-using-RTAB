#!/usr/bin/env bash
# RGB-D + /odom sanity checks (run while full_system is up).
set -euo pipefail

echo "=== Topics (odom / rgbd / camera) ==="
ros2 topic list | grep -E "odom|rgbd|camera" || true

echo ""
echo "=== /odom type ==="
ros2 topic info /odom -v 2>/dev/null | head -n 20 || echo "WARN: /odom not found"

echo ""
echo "=== Rates (Ctrl+C after ~5 s if needed) ==="
timeout 8 ros2 topic hz /rgbd_camera/image 2>/dev/null || echo "WARN: /rgbd_camera/image"
timeout 8 ros2 topic hz /rgbd_camera/depth_image 2>/dev/null || echo "WARN: /rgbd_camera/depth_image"
timeout 8 ros2 topic hz /rgbd_camera/camera_info 2>/dev/null || echo "WARN: /rgbd_camera/camera_info"
timeout 8 ros2 topic hz /odom 2>/dev/null || echo "WARN: /odom not publishing"

echo ""
echo "=== One-shot /odom + VO inliers ==="
timeout 3 ros2 topic echo /odom --once 2>/dev/null || echo "WARN: no /odom message"
timeout 3 ros2 topic echo /rtabmap/odom_info --field inliers --once 2>/dev/null || true

echo ""
echo "=== TF chain ==="
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -n 12 || echo "WARN: odom->base_link"
timeout 4 ros2 run tf2_ros tf2_echo base_link rgbd_camera 2>/dev/null | head -n 12 || echo "WARN: base_link->rgbd_camera"

echo ""
echo "=== Optional slow VO motion (odom-only; not with avoidance) ==="
echo "  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p linear_x:=0.02"
