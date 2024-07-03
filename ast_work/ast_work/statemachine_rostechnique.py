### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
from rclpy.node import Node
import smach

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(Node):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        super().__init__('state_machine_battery')
        self.battery_voltage = 24.0
        self.battery_percentage = 100.0
        self.discharging_factor = 0.99
        self.battery_threshold = 30.0
        self.collision_threshold = 1.0

        self.set_battery_publisher = self.create_publisher(Float32, '/battery_status',10)
        self.battery_timer = self.create_timer(0.3,self.execute)
 
    def execute(self):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        #raise NotImplementedError()    
        #data = str(self.battery_voltage)
        msg = Float32()
        msg.data = self.battery_voltage
        self.set_battery_publisher.publish(msg)
        self.get_logger().info(f'Battery Percentage {msg.data}')

        self.battery_voltage = self.battery_voltage * self.discharging_factor
        #self.battery_percentage = self.battery_percentage * self.discharging_factor

# TODO: define any additional states if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    rclpy.init(args=args)
    node = MonitorBatteryAndCollision()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()