#!/usr/bin/env python
"""
AMCL Hook for io-gita Zone Hints — ROS1 Version
=================================================

THIS IS THE ONLY FILE YOU NEED TO ADD TO YOUR SYSTEM.

Add this to your AMCL launch. It subscribes to /iogita/zone_hint
and publishes a constrained /initialpose to speed up AMCL convergence.

Usage (ROS1):
    rosrun your_package amcl_hook_example.py

Usage (ROS2):
    See the ROS2 section at the bottom of this file.

Total: ~30 lines of actual code.
"""

import json

# ═══════════════════════════════════════════════════════════
# ROS1 VERSION
# ═══════════════════════════════════════════════════════════

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

# ── CONFIGURE THIS: Your zone → map coordinate mapping ──
# These must match your warehouse_config.yaml zones
# x, y = center of zone in map frame (meters)
# radius = search area for AMCL particles (meters)

ZONE_COORDINATES = {
    # ← CHANGE ALL OF THESE to your actual map coordinates
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


def on_zone_hint(msg):
    """
    Receive io-gita zone hint and publish constrained /initialpose.

    This is called ONCE during cold start. It tells AMCL to search
    only in the hinted zones instead of the entire map.
    """
    zones = json.loads(msg.data)
    rospy.loginfo(f"[io-gita] Zone hint received: {zones}")

    for zone_name in zones:
        if zone_name in ZONE_COORDINATES:
            region = ZONE_COORDINATES[zone_name]

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            # Set position to zone center
            pose.pose.pose.position.x = region["x"]
            pose.pose.pose.position.y = region["y"]
            pose.pose.pose.position.z = 0.0

            # Orientation: identity quaternion (let AMCL figure out heading)
            pose.pose.pose.orientation.w = 1.0

            # Covariance: constrain search to this zone's area
            # AMCL will only spread particles within this radius
            cov = region["radius"] ** 2
            pose.pose.covariance[0]  = cov    # x variance
            pose.pose.covariance[7]  = cov    # y variance
            pose.pose.covariance[35] = 1.0    # theta variance (radians^2)

            initial_pose_pub.publish(pose)
            rospy.loginfo(f"[io-gita] Published /initialpose at "
                          f"({region['x']}, {region['y']}) "
                          f"radius={region['radius']}m for zone {zone_name}")
            break  # Publish only the first valid zone hint


def on_zone_change(msg):
    """Optional: log zone changes for fleet dashboard."""
    rospy.loginfo(f"[io-gita] Current zone: {msg.data}")


def on_confidence(msg):
    """Optional: warn if confidence is low."""
    if msg.data < 0.5:
        rospy.logwarn(f"[io-gita] Low zone confidence: {msg.data:.2f}")


if __name__ == "__main__":
    rospy.init_node("iogita_amcl_hook")

    # Publisher: constrained initial pose for AMCL
    initial_pose_pub = rospy.Publisher(
        "/initialpose", PoseWithCovarianceStamped, queue_size=1)

    # Subscriber: io-gita zone hints (for cold start)
    rospy.Subscriber("/iogita/zone_hint", String, on_zone_hint)

    # Optional subscribers: zone tracking and confidence
    rospy.Subscriber("/iogita/zone", String, on_zone_change)
    rospy.Subscriber("/iogita/zone_confidence", Float32, on_confidence)

    rospy.loginfo("[io-gita] AMCL hook node started. Waiting for zone hints...")
    rospy.spin()


# ═══════════════════════════════════════════════════════════
# ROS2 VERSION (alternative — uncomment if using ROS2)
# ═══════════════════════════════════════════════════════════
#
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32
# from geometry_msgs.msg import PoseWithCovarianceStamped
#
# class AmclIoGitaHook(Node):
#     def __init__(self):
#         super().__init__("iogita_amcl_hook")
#         self.pub = self.create_publisher(
#             PoseWithCovarianceStamped, "/initialpose", 10)
#         self.create_subscription(
#             String, "/iogita/zone_hint", self.on_zone_hint, 10)
#         self.create_subscription(
#             String, "/iogita/zone", self.on_zone_change, 10)
#         self.create_subscription(
#             Float32, "/iogita/zone_confidence", self.on_confidence, 10)
#         self.get_logger().info("io-gita AMCL hook started")
#
#     def on_zone_hint(self, msg):
#         zones = json.loads(msg.data)
#         self.get_logger().info(f"Zone hint: {zones}")
#         for zone_name in zones:
#             if zone_name in ZONE_COORDINATES:
#                 region = ZONE_COORDINATES[zone_name]
#                 pose = PoseWithCovarianceStamped()
#                 pose.header.frame_id = "map"
#                 pose.header.stamp = self.get_clock().now().to_msg()
#                 pose.pose.pose.position.x = region["x"]
#                 pose.pose.pose.position.y = region["y"]
#                 pose.pose.pose.orientation.w = 1.0
#                 cov = region["radius"] ** 2
#                 pose.pose.covariance[0] = cov
#                 pose.pose.covariance[7] = cov
#                 pose.pose.covariance[35] = 1.0
#                 self.pub.publish(pose)
#                 break
#
#     def on_zone_change(self, msg):
#         self.get_logger().info(f"Zone: {msg.data}")
#
#     def on_confidence(self, msg):
#         if msg.data < 0.5:
#             self.get_logger().warn(f"Low confidence: {msg.data:.2f}")
#
# def main():
#     rclpy.init()
#     node = AmclIoGitaHook()
#     rclpy.spin(node)
#     rclpy.shutdown()
