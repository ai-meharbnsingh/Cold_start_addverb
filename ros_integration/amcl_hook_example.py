#!/usr/bin/env python
"""
AMCL Hook for io-gita Zone Hints — ROS1 + ROS2 Compatible
===========================================================

THIS IS THE ONLY FILE YOU NEED TO ADD TO YOUR SYSTEM.

Subscribes to /iogita/zone_hint → publishes constrained /initialpose
to speed up AMCL convergence during cold start.

Auto-detects ROS1 (rospy) or ROS2 (rclpy). No code changes needed.

Usage (ROS1):  rosrun your_package amcl_hook_example.py
Usage (ROS2):  ros2 run your_package amcl_hook_example

Total: ~30 lines of actual code. The rest is boilerplate.

Contact: ai.meharbansingh@gmail.com
"""

import json

# ── CONFIGURE THIS: Your zone -> map coordinate mapping ──
# These must match your warehouse_config.yaml zones
# x, y = center of zone in map frame (meters)
# radius = search area for AMCL particles (meters)

ZONE_COORDINATES = {
    # CHANGE ALL OF THESE to your actual map coordinates
    "DOCK_A":   {"x":  5.0, "y":  5.0, "radius": 5.0},
    "AISLE_1":  {"x": 15.0, "y":  5.0, "radius": 3.0},
    "CROSS_N":  {"x": 25.0, "y":  5.0, "radius": 4.0},
    "AISLE_2":  {"x": 35.0, "y":  5.0, "radius": 3.0},
    "DOCK_B":   {"x": 45.0, "y":  5.0, "radius": 5.0},
    "LANE_W":   {"x":  5.0, "y": 15.0, "radius": 3.0},
    "SHELF_1":  {"x": 15.0, "y": 15.0, "radius": 3.0},
    "MID_N":    {"x": 25.0, "y": 15.0, "radius": 3.0},
    "SHELF_2":  {"x": 35.0, "y": 15.0, "radius": 3.0},
    "LANE_E":   {"x": 45.0, "y": 15.0, "radius": 3.0},
    "CROSS_W":  {"x":  5.0, "y": 25.0, "radius": 4.0},
    "SHELF_3":  {"x": 15.0, "y": 25.0, "radius": 3.0},
    "HUB":      {"x": 25.0, "y": 25.0, "radius": 5.0},
    "SHELF_4":  {"x": 35.0, "y": 25.0, "radius": 3.0},
    "CROSS_E":  {"x": 45.0, "y": 25.0, "radius": 4.0},
    "LANE_W2":  {"x":  5.0, "y": 35.0, "radius": 3.0},
    "SHELF_5":  {"x": 15.0, "y": 35.0, "radius": 3.0},
    "MID_S":    {"x": 25.0, "y": 35.0, "radius": 3.0},
    "SHELF_6":  {"x": 35.0, "y": 35.0, "radius": 3.0},
    "LANE_E2":  {"x": 45.0, "y": 35.0, "radius": 3.0},
    "DOCK_C":   {"x":  5.0, "y": 45.0, "radius": 5.0},
    "AISLE_3":  {"x": 15.0, "y": 45.0, "radius": 3.0},
    "CROSS_S":  {"x": 25.0, "y": 45.0, "radius": 4.0},
    "AISLE_4":  {"x": 35.0, "y": 45.0, "radius": 3.0},
    "DOCK_D":   {"x": 45.0, "y": 45.0, "radius": 5.0},
}


# ═══════════════════════════════════════════════════════════
# CORE LOGIC (same for ROS1 and ROS2)
# ═══════════════════════════════════════════════════════════

def build_initial_pose(zone_name, stamp_fn, frame_id="map"):
    """Build a PoseWithCovarianceStamped for the given zone.
    Returns None if zone not in ZONE_COORDINATES."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    if zone_name not in ZONE_COORDINATES:
        return None

    region = ZONE_COORDINATES[zone_name]
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp_fn()
    pose.pose.pose.position.x = region["x"]
    pose.pose.pose.position.y = region["y"]
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.w = 1.0
    cov = region["radius"] ** 2
    pose.pose.covariance[0]  = cov    # x variance
    pose.pose.covariance[7]  = cov    # y variance
    pose.pose.covariance[35] = 1.0    # theta variance (radians^2)
    return pose


# ═══════════════════════════════════════════════════════════
# AUTO-DETECT ROS VERSION AND RUN
# ═══════════════════════════════════════════════════════════

def run_ros1():
    """ROS1 (rospy) version."""
    import rospy
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rospy.init_node("iogita_amcl_hook")
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    def on_zone_hint(msg):
        zones = json.loads(msg.data)
        rospy.loginfo(f"[io-gita] Zone hint received: {zones}")
        for zone_name in zones:
            pose = build_initial_pose(zone_name, rospy.Time.now)
            if pose is not None:
                pub.publish(pose)
                rospy.loginfo(f"[io-gita] Published /initialpose for zone {zone_name}")
                break

    def on_zone_change(msg):
        rospy.loginfo(f"[io-gita] Current zone: {msg.data}")

    def on_confidence(msg):
        if msg.data < 0.5:
            rospy.logwarn(f"[io-gita] Low zone confidence: {msg.data:.2f}")

    rospy.Subscriber("/iogita/zone_hint", String, on_zone_hint)
    rospy.Subscriber("/iogita/zone", String, on_zone_change)
    rospy.Subscriber("/iogita/zone_confidence", Float32, on_confidence)
    rospy.loginfo("[io-gita] AMCL hook started (ROS1). Waiting for zone hints...")
    rospy.spin()


def run_ros2():
    """ROS2 (rclpy) version."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import PoseWithCovarianceStamped

    class AmclIoGitaHook(Node):
        def __init__(self):
            super().__init__("iogita_amcl_hook")
            self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            self.create_subscription(String, "/iogita/zone_hint", self.on_zone_hint, 10)
            self.create_subscription(String, "/iogita/zone", self.on_zone_change, 10)
            self.create_subscription(Float32, "/iogita/zone_confidence", self.on_confidence, 10)
            self.get_logger().info("[io-gita] AMCL hook started (ROS2). Waiting for zone hints...")

        def on_zone_hint(self, msg):
            zones = json.loads(msg.data)
            self.get_logger().info(f"Zone hint received: {zones}")
            for zone_name in zones:
                pose = build_initial_pose(zone_name, lambda: self.get_clock().now().to_msg())
                if pose is not None:
                    self.pub.publish(pose)
                    self.get_logger().info(f"Published /initialpose for zone {zone_name}")
                    break

        def on_zone_change(self, msg):
            self.get_logger().info(f"Current zone: {msg.data}")

        def on_confidence(self, msg):
            if msg.data < 0.5:
                self.get_logger().warn(f"Low zone confidence: {msg.data:.2f}")

    rclpy.init()
    node = AmclIoGitaHook()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    # Auto-detect: try ROS2 first, fall back to ROS1
    try:
        import rclpy
        run_ros2()
    except ImportError:
        try:
            import rospy
            run_ros1()
        except ImportError:
            print("ERROR: Neither rclpy (ROS2) nor rospy (ROS1) found.")
            print("Install ROS1 Noetic or ROS2 Humble/Iron first.")
            exit(1)
