import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        self.move_sub = self.create_subscription(String , '/move', self.call_back, 10)
        #self.move_pub = self.create_publisher(String , '/move_status', 10)
        self.counter = 0
        self.get_logger().info("Waiting for Action from Client")

    def call_back (self, msg):
        
        if msg:
            self.get_logger().info(f'{msg.data} ')
            for i in range(5):
                self.counter = self.counter + i
                self.get_logger().info(f'{self.counter}')

        
            


def main(args=None)-> None:
    rclpy.init(args=args)
    node=Move()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

