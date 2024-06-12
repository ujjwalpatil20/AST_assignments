import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Robot



class MoveActionClient(Node):

    def __init__(self):
        super().__init__('Move_action_client')
        self._action_client = ActionClient(self, Robot, '/move')

    def send_goal(self, order):
        goal_msg = Robot.Goal()
        result = Robot.Result()
        goal_msg.data = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = MoveActionClient()

    future = action_client.send_goal('10')

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()