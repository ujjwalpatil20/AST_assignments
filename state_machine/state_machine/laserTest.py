import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# Global variable to hold the minimum distance
min_distance = None

def laser_scan_callback(msg):
    global min_distance
    min_distance = min(msg.ranges)
    rclpy.logging.get_logger('laser_scan_callback').info(f'Minimum distance: {min_distance}')

class DistanceProcessor:
    def __init__(self, node):
        self.node = node

    def process_distance(self):
        global min_distance
        if min_distance is not None:
            self.node.get_logger().info(f'Processing distance: {min_distance}')
            # Implement your logic here
            self.node.get_logger().info(f'Using distance: {min_distance}')

def main(args=None):
    global min_distance

    rclpy.init(args=args)
    node = Node('laser_scan_processor')
    
    # Create the subscription
    node.create_subscription(LaserScan, 'scan', laser_scan_callback, 10)
    
    distance_processor = DistanceProcessor(node)

    def timer_callback():
        distance_processor.process_distance()

    timer_period = 1.0  # seconds
    node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
