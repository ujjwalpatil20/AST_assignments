#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class SimpleNavigationNode(Node):
    def __init__(self):
        super().__init__('simple_navigation_node')

        # Subscriber to the goal_pose topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Subscriber to the odom topic
        self.odom_sub = self.create_subscription(
            Odometry,
            'robile_0/odom',
            self.odom_callback,
            10
        )

        # Publisher to the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'robile_0/cmd_vel', 10)

        self.goal_pose = None
        self.current_pose = None
        self.reached_goal = False

        self.timer = self.create_timer(0.1, self.update)

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.reached_goal = False
        self.get_logger().info(f"Received goal: {self.goal_pose.position.x}, {self.goal_pose.position.y}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def update(self):
        if self.goal_pose is None or self.current_pose is None:
            return

        if self.reached_goal:
            # Stop the robot if goal is reached
            self.publish_cmd_vel(0.0, 0.0)
            return

        # Calculate distance to the goal
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Calculate angle to the goal
        goal_yaw = math.atan2(dy, dx)

        # Current orientation (yaw) from odometry
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate angle difference
        angle_diff = self.normalize_angle(goal_yaw - current_yaw)

        # Threshold for distance and angle to consider the goal reached
        distance_threshold = 0.1  # meters
        angle_threshold = 0.1     # radians

        # If the robot is near the goal position and oriented correctly, stop
        if distance < distance_threshold and abs(angle_diff) < angle_threshold:
            self.reached_goal = True
            self.get_logger().info("Goal reached")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # Simple proportional controller for linear and angular velocity
        linear_velocity = 0.5 * distance if distance > distance_threshold else 0.0
        angular_velocity = 1.0 * angle_diff if abs(angle_diff) > angle_threshold else 0.0

        self.publish_cmd_vel(linear_velocity, angular_velocity)

    def get_yaw_from_quaternion(self, orientation):
        # Converts quaternion to Euler angles (yaw)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Normalize angle to the range [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_cmd_vel(self, linear, angular):
        # Publishes the command velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
