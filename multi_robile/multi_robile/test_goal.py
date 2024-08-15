import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
import numpy as np


class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')
        self.publisher_ = self.create_publisher(Twist, '/robile_0/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/robile_0/odom', self.odom_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self.state = [True, False, False]
        self.stop_state = False

        self.rotate_speed = 1.0
        self.threshold = 0.2  # How close to get to the goal position and orientation
        self.distance_threshold = 0.5

        # self.goal_x, self.goal_y, self.goal_theta = 2.0, -3.0, -2.0 # Goal Position
        self.position_x, self.position_y, self.orientation = 0.0, 0.0, 0.0  # Initial x, y, theta in radians
        
        self.w = 0.0 # For storing the omega value of the odom
        self.goal_direction = 0.0
        self.initial_rotation_direction = "left"
        self.final_rotation_direction = "left"
        self.final_orientation = 0.0
        self.rotating_angle = 0.0
        
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.new_pos_x = 0.0
        self.new_pos_y = 0.0


    def odom_callback(self, msg):
        self.w = msg.pose.pose.orientation.w
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

    def goal_callback(self, data):
        pose_stamp = data.pose.position
        # self.goal_pose = self.rescale_pose_to_mapsize(pose_stamp, self.grid)
        self.get_logger().info(f'Goal_pose in grid:{self.goal_pose_sub}')
        self.get_logger().info(f"Getting goal pose: {pose_stamp}")
        
        self.goal_x = pose_stamp.x 
        self.goal_y = pose_stamp.y
        self.goal_theta = 0.0

        self.rotate_to_goal()
    
    def rotate_right(self, rad_ang):
        return 1+(rad_ang/pi)
    
    def rotate_left(self, rad_ang):
        return 1-(rad_ang/pi)
    
    # For finding either to rotate left or right for first task
    def left_right(self, goal_direction):
        if goal_direction < 0:
            print("Rotating Right")
            initial_rotation_direction = "right"
            goal_direction = self.rotate_right(goal_direction)
            return initial_rotation_direction, goal_direction
        else:
            print("Rotating Left")
            initial_rotation_direction = "left"
            goal_direction = self.rotate_left(goal_direction)
            return initial_rotation_direction, goal_direction
        
    # For finding either to rotate left or right for final task
    def final_left_right(self, goal_direction):
        if goal_direction < 0:
            print("Rotating Left")
            initial_rotation_direction = "left"
            goal_direction = self.rotate_right(goal_direction)
            return initial_rotation_direction, goal_direction
        else:
            print("Rotating Right")
            initial_rotation_direction = "right"
            goal_direction = self.rotate_left(goal_direction)
            return initial_rotation_direction, goal_direction
    
    # Function to find the angle to which robot has to rotate to face the goal and find the goal pose relative to odom
    def find_goal_data(self):
        # Getting the Angle  till where to rotate initially
        self.goal_direction = atan2(self.goal_y - self.position_y, self.goal_x - self.position_x)
        print(f"Goal_Direction_rad = {self.goal_direction}")
        print(f"Goal_Direction_deg = {np.rad2deg(self.goal_direction)}")
        self.initial_rotation_direction, self.goal_direction = self.left_right(self.goal_direction)

        # Getting the distance from odom origin to goal pos
        self.new_pos_x = self.current_pos_x + self.goal_x
        self.new_pos_y = self.current_pos_y + self.goal_y

        print(f"self.new_position = {self.new_pos_x}")
    
    def rotate_to_goal(self):
        if not self.goal_direction:
            self.find_goal_data()
        
        if self.state[0]:
            print(f"Goal_direction = {self.goal_direction}, w = {self.w}, remaining_distance = {abs(self.w - self.goal_direction)}")
            if abs(self.w - self.goal_direction) > self.threshold:
                twist = Twist()
                if self.initial_rotation_direction == "right":
                    twist.angular.z = -self.rotate_speed
                else:
                    twist.angular.z = self.rotate_speed
                self.publisher_.publish(twist)
            else:
                twist = Twist()
                self.publisher_.publish(twist)
                self.state[0] = False
                self.state[1] = True

        if self.state[1]:
            print("*"*20)
            print(f"self.new_pos_x = {self.new_pos_x}, Current_Pos_x = {self.current_pos_x}, remaining_distance_x = {abs(self.new_pos_x - self.current_pos_x)}")
            print(f"self.new_pos_y = {self.new_pos_y}, Current_Pos_y = {self.current_pos_y}, remaining_distance_y = {abs(self.new_pos_y - self.current_pos_y)}")
            print("*"*20)
            if (abs(self.new_pos_x - self.current_pos_x) > self.distance_threshold) or (abs(self.new_pos_y - self.current_pos_y) > self.distance_threshold):
                twist = Twist()
                twist.linear.x = 0.5
                self.publisher_.publish(twist)
            else:
                twist = Twist()
                self.publisher_.publish(twist)
                self.state[1] = False
                self.state[2] = True

        if self.state[2]:
            self.final_rotation_direction, self.final_orientation = self.final_left_right(self.goal_theta)
            if abs(self.w - self.final_orientation) > self.threshold:
                twist = Twist()
                if self.final_rotation_direction == "right":
                    twist.angular.z = -self.rotate_speed
                else:
                    twist.angular.z = self.rotate_speed
                self.publisher_.publish(twist)
            else:
                twist = Twist()
                self.publisher_.publish(twist)
                self.get_logger().info(f'Goal reached !!!')
                self.state[2] = False   

def main(args=None):
    rclpy.init(args=args)
    node = OdometryMotionModel()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        # node.rotate_to_goal()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()