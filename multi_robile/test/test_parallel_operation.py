import unittest
from unittest.mock import Mock, patch
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from multi_robile.parallel_operation import FormationController

class TestFormationController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch.object(FormationController, 'create_subscription'), \
             patch.object(FormationController, 'create_publisher', return_value=Mock()):
            self.node = FormationController()
            self.node.publisher_robot_1 = Mock()
            self.node.publisher_robot_2 = Mock()

    def test_initialization(self):
        self.assertIsInstance(self.node, FormationController)
        self.assertEqual(self.node.distance, 2.0)

    def test_robot_0_cmd_vel_callback(self):
        test_msg = Twist()
        test_msg.linear.x = 1.5
        test_msg.angular.z = 0.2
        self.node.robot_0_cmd_vel_callback(test_msg)
        self.assertEqual(self.node.last_cmd.linear.x, 1.5)
        self.assertEqual(self.node.last_cmd.angular.z, 0.2)

    def test_robot_0_odom_callback(self):
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 2.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0  # No rotation
        self.node.robot_0_odom_callback(odom_msg)
        self.assertEqual(self.node.robot_0_pos, (1.0, 2.0, 0.0))

    def test_adjust_robot_velocity(self):
        self.node.last_cmd = Twist()
        self.node.last_cmd.linear.x = 1.0

        leader_pos = (2.0, 2.0, 0)
        robot_pos = (0.0, 0.0, 0)
        self.node.adjust_robot_velocity(1, robot_pos, leader_pos, self.node.publisher_robot_1)

        # Calculate expected command
        dx, dy = 2.0, 2.0
        distance_error = ((dx ** 2 + dy ** 2) ** 0.5) - self.node.distance
        expected_cmd = Twist()
        expected_cmd.linear.x = 1.0 + 0.1 * distance_error
        expected_cmd.angular.z = 0.0

        self.node.publisher_robot_1.publish.assert_called_with(expected_cmd)

if __name__ == '__main__':
    unittest.main()
