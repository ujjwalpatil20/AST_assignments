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

class Laser_Data(Node):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        super().__init__('state_machine_laser')
        self.laser_data_sub = self.create_subscription(LaserScan, '/scan', self.execute, 10)
 
    def execute(self,msg):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        #raise NotImplementedError()
        ranges_data = msg.ranges
        self.get_logger().info(f'Ranges_Data Percentage {min(ranges_data)}')
        
# TODO: define any additional states if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    rclpy.init(args=args)
    node = Laser_Data()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()