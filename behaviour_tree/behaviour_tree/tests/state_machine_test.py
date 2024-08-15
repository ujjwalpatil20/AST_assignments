import unittest
from unittest.mock import MagicMock
import rclpy
from behaviour_tree.statemachine import MonitorBatteryAndCollision, RotateBase, StopMotion

class TestStateMachine(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_monitor_battery_and_collision_initialization(self):
        state = MonitorBatteryAndCollision(self.node)
        self.assertEqual(state.battery_percentage, 100)
        self.assertEqual(state.collision_threshold, 0.7)

    def test_rotate_base_initialization(self):
        state = RotateBase(self.node)
        self.assertEqual(state.rotation_speed, 0.75)

    def test_stop_motion_initialization(self):
        state = StopMotion(self.node)
        self.assertTrue(state.collide)

    def test_monitor_battery_and_collision_execution(self):
        state = MonitorBatteryAndCollision(self.node)
        state.laser_callback = MagicMock()
        state.set_battery_publisher = MagicMock()

        # Mock userdata with battery_update
        userdata = MagicMock()
        userdata.battery_update = 50
        result = state.execute(userdata)
        self.assertEqual(result, 'continue_monitoring')

    def test_rotate_base_execution(self):
        state = RotateBase(self.node)
        state.cmd_vel_pub = MagicMock()

        # Mock userdata with battery_level
        userdata = MagicMock()
        userdata.battery_level = 50
        result = state.execute(userdata)
        self.assertEqual(result, 'low_battery')

    def test_stop_motion_execution(self):
        state = StopMotion(self.node)
        state.cmd_vel_pub = MagicMock()
        state.laser_callback = MagicMock()

        # Mock laser data
        state.scan = 0.8  # No collision
        result = state.execute(None)
        self.assertEqual(result, 'continue_monitoring')

        # Mock laser data
        state.scan = 0.5  # Collision
        result = state.execute(None)
        self.assertEqual(result, 'stop')

if __name__ == '__main__':
    unittest.main()
