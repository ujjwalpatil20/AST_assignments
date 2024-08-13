import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        self.distance = 2.0  # desired distance between robots in meters
        
        # Subscribers for odometry and cmd_vel of robot 0
        self.create_subscription(Odometry, '/robile_0/odom', self.robot_0_odom_callback, 10)
        self.create_subscription(Twist, '/robile_0/cmd_vel', self.robot_0_cmd_vel_callback, 10)
        
        # Odometry storage
        self.robot_0_pos = None
        self.robot_1_pos = None

        # Publishers for robot 1 and robot 2
        self.publisher_robot_1 = self.create_publisher(Twist, '/robile_1/cmd_vel', 10)
        self.publisher_robot_2 = self.create_publisher(Twist, '/robile_2/cmd_vel', 10)

        # Subscribers for robot 1 and robot 2 odometry
        self.create_subscription(Odometry, '/robile_1/odom', self.robot_1_odom_callback, 10)
        self.create_subscription(Odometry, '/robile_2/odom', self.robot_2_odom_callback, 10)

        # Last known cmd_vel from robot 0
        self.last_cmd = Twist()

    def robot_0_cmd_vel_callback(self, msg):
        self.last_cmd = msg

    def robot_0_odom_callback(self, msg):
        self.robot_0_pos = self.extract_position(msg)

    def robot_1_odom_callback(self, msg):
        self.robot_1_pos = self.extract_position(msg)
        self.adjust_robot_velocity(1, self.robot_1_pos, self.robot_0_pos, self.publisher_robot_1)

    def robot_2_odom_callback(self, msg):
        self.robot_2_pos = self.extract_position(msg)
        self.adjust_robot_velocity(2, self.robot_2_pos, self.robot_1_pos, self.publisher_robot_2)

    def extract_position(self, odom_msg):
        # Extract position and orientation from the odometry message
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return (position.x, position.y, yaw)

    def adjust_robot_velocity(self, robot_id, robot_pos, leader_pos, publisher):
        if leader_pos is None or robot_pos is None:
            return
        # Simple control law to maintain distance: proportional controller
        dx = leader_pos[0] - robot_pos[0]
        dy = leader_pos[1] - robot_pos[1]
        distance_error = (dx**2 + dy**2)**0.5 - self.distance

        # Create a new Twist message
        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x + 0.1 * distance_error
        cmd.angular.z = self.last_cmd.angular.z
        publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    formation_controller = FormationController()
    rclpy.spin(formation_controller)
    formation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
