### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
from rclpy.node import Node
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class RotateBase(Node):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        super().__init__('state_machine')
        self.rotation_speed = 1
        self.cmd_vel_pub = self.create_publisher(Twist , "/cmd_vel",10)
        self.rotation = self.create_timer(0.3,self.execute)
 
    def execute(self):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        #raise NotImplementedError()    
        #data = str(self.battery_voltage)
        msg = Twist()
        msg.angular.z = 1.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Rotation at z {msg.angular.z}')


# TODO: define any additional states if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    rclpy.init(args=args)
    node = RotateBase()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()