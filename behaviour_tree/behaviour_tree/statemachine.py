### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
from rclpy.node import Node
import smach # specifically for ros1
import smach_ros

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]
class MonitorBatteryAndCollision(smach_ros.RosState):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        smach_ros.RosState.__init__(self, node,
                                    outcomes = ['low_battery', 'stop', 'continue_monitoring'],
                                    input_keys = ['battery_update'],
                                    output_keys = ['battery_level']
                                    )
        self.screentimer = 0
        self.node_ = node

        self.discharging_factor = 0.01
        self.battery_threshold = 30
        self.collision_threshold = 0.7
        self.battery_percentage = 100
        
        self.var= 1
        self.scan = 50
        self.collide = False
        self.battery = True

        self.cmd_vel_pub = self.node.create_publisher(Twist , "/cmd_vel",10)
        self.laser_data_sub = self.node.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.set_battery_publisher = self.node.create_publisher(String, '/battery_status',10)

    def laser_callback(self,  msg):
        #shorten the scope of laser (Todo)
        ranges_data = msg.ranges
        self.scan = min(ranges_data)
        self.node.get_logger().info('='*50) 
        self.node.get_logger().info('')
        self.node.get_logger().info(f'INSIDE LOOP {self.scan}')
        self.node.get_logger().info('')
        self.node.get_logger().info('='*50)
        
    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        #raise NotImplementedError()
        rclpy.spin_once(self.node_)
        if 'battery_update' in userdata and self.var != 0:
            self.battery_percentage = userdata.battery_update
            self.node.get_logger().info('='*50)
            self.node.get_logger().info('')
            self.node.get_logger().info('Data received by another state')
            self.node.get_logger().info('')
            self.node.get_logger().info('='*50)
            self.var = self.var - 1

        msg = String()
        msg.data = str(self.battery_percentage)
        self.set_battery_publisher.publish(msg)
        
        
        #self.node.get_logger().info(f'Scanner Values:  {self.scan}')

        self.battery_percentage = self.battery_percentage - self.discharging_factor
        self.node.get_logger().info(f'Battery Percentage {self.battery_percentage}')
        
        if self.scan < self.collision_threshold:
            self.node.get_logger().info(f'Collision nearby at {self.scan}')
            self.collide = True
            return 'stop'
        if self.battery_percentage < self.battery_threshold:
            userdata.battery_level = self.battery_percentage
            self.battery_percentage = 100   
            return 'low_battery'
        
        return 'continue_monitoring'
        

class RotateBase(smach_ros.RosState):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        #raise NotImplementedError()
        smach_ros.RosState.__init__(self, node,
                                    outcomes=['low_battery','stop', 'continue_monitoring'],
                                    input_keys = ['battery_level'],
                                    output_keys = ['battery_update']
                                    )
        self.rotation_speed = 0.75
        self.var = 1
        self.charging_factor = 0.01
        self.battery_ = 0
        self.cmd_vel_pub = self.node.create_publisher(Twist , "/cmd_vel",10)

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        #raise NotImplementedError()
        
        if 'battery_level' in userdata and self.var != 0:
                    self.battery_ = userdata.battery_level
                    self.node.get_logger().info('='*50)
                    self.node.get_logger().info('')
                    self.node.get_logger().info('Data received by state A')
                    self.node.get_logger().info('')
                    self.node.get_logger().info('='*50)
                    self.var = self.var - 1

        roll = Twist()
        roll.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(roll)
        
        self.battery_ = self.battery_ + self.charging_factor
        self.node.get_logger().info(f'Battery Percentage {self.battery_}')
        
        if  self.battery_ >= 100:
            roll = Twist()
            roll.angular.z = 0.0  # Stop rotation
            self.cmd_vel_pub.publish(roll)
            self.battery_ = 0
            return 'continue_monitoring'
        else:
            return 'low_battery'

class StopMotion(smach_ros.RosState):
    """State to stop the robot's motion
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        #raise NotImplementedError()
        smach_ros.RosState.__init__(self, node,
                                    outcomes=['stop','low_battery','continue_monitoring'])
        self.collision_threshold = 0.7
        self.collide = True
        self.node_ = node

        self.cmd_vel_pub = self.node.create_publisher(Twist , "/cmd_vel",10)
        self.laser_data_sub = self.node.create_subscription(LaserScan, '/scan', self.laser_callback , 10)

    def laser_callback(self,  msg):
        ranges_data = msg.ranges
        self.scan = min(ranges_data)
        self.node.get_logger().info('='*50)
        self.node.get_logger().info('')
        self.node.get_logger().info('Laser Call back Loop')
        self.node.get_logger().info('')
        self.node.get_logger().info('='*50)

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        rclpy.spin_once(self.node_)
        self.node.get_logger().info('I am stopp !')
        roll = Twist()
        roll.linear.x = 0.0
        roll.linear.y = 0.0
        roll.linear.z = 0.0
        roll.angular.x = 0.0
        roll.angular.y = 0.0
        roll.angular.z = 0.0
        self.cmd_vel_pub.publish(roll)
        
        if self.scan > self.collision_threshold:
            self.node.get_logger().info(f'No Collision')
            self.collide = False    
            return 'continue_monitoring'
        else:
            return 'stop'
    

# TODO: define any additional states if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """
     
    rclpy.init(args=args)
    node = rclpy.create_node('state_machine_node')
    # TODO: initialise a ROS2 node, set any threshold values, and define the state machine
    # YOUR CODE HERE
    #raise NotImplementedError()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['complete'])
    
    # Open the container
    with sm:
    # Add states to the container
        smach.StateMachine.add('MBCOLLISION', MonitorBatteryAndCollision(node), 
                               transitions={'low_battery': 'ROTATEBASE',#'ROTATEBASE', 
                                            'stop': 'STOPMOTION',#'STOPMOTION',
                                            'continue_monitoring': 'MBCOLLISION'})
        smach.StateMachine.add('ROTATEBASE', RotateBase(node), 
                                transitions={'continue_monitoring':'MBCOLLISION',#MBCOLLISION
                                             'stop': 'STOPMOTION' ,#STOPMOTION 
                                             'low_battery': 'ROTATEBASE'})
        smach.StateMachine.add('STOPMOTION', StopMotion(node),
                                transitions={'stop': 'STOPMOTION',
                                             'continue_monitoring':'MBCOLLISION',
                                             'low_battery':'ROTATEBASE'})
    # Execute SMACH plan
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()
    
    outcome = sm.execute()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()