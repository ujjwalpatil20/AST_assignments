import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Place(Node):
    
    def __init__(self):
        super().__init__('place_node')
    
        self.place_sub = self.create_subscription(String , '/place', self.call_back, 10)
        self.get_logger().info("Waiting for Action from Client")

    def call_back (self, msg):
        if msg:
            self.get_logger().info(f'{msg.data} ')

def main(args=None)-> None:
    rclpy.init(args=args)
    node=Place()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
