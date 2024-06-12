import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Robot
from std_msgs.msg import String
import time

class TestActionServer(Node):

    def __init__(self):
        super().__init__('test_action_server')
        self._action_server = ActionServer(self,
                                           Robot,
                                           'move',
                                           self.execute_callback)
        self.publishing_move = self.create_publisher(String, '/move', 10)
        self.publishing_perceive = self.create_publisher(String, '/perceive_plane', 10)
        self.publishing_pick = self.create_publisher(String, '/pick', 10)
        self.publishing_place = self.create_publisher(String, '/place', 10)
        
        self.msg = String()
        self.robot_plan = [False, False, False, False] #[Move, Perceive, Pick, Place]
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Robot Initializating')
        
        result = Robot.Result()
        feedbacks =  Robot.Feedback()

        self.robot_plan[0] = True
        time.sleep(5)
        feedbacks.feedback_data = 'Executing Move .....'
        goal_handle.publish_feedback(feedbacks)
        self.msg.data = 'Moving Robot ...'
        self.publishing_move.publish(self.msg)
        time.sleep(10)

        if self.robot_plan[0]:
            self.msg.data = 'Move Action Completed'
            self.get_logger().info('Move operation Done')
            feedbacks.feedback_data = 'Move Operation Done'
            goal_handle.publish_feedback(feedbacks)
            self.publishing_move.publish(self.msg)
            time.sleep(1)
            self.get_logger().info('Perceiving Plane.....')
            self.msg.data = 'Perceiving Plane to pick the object ...'
            self.publishing_perceive.publish(self.msg)
            time.sleep(10)
            self.robot_plan[1], self.robot_plan[0] = True , False

        if self.robot_plan[1]:
            self.msg.data = 'Perceiving Action Completed'
            self.get_logger().info('Perceiving Operation Done')
            self.publishing_perceive.publish(self.msg)
            time.sleep(1)
            self.get_logger().info('Picking .....')
            self.msg.data = 'Picking the object ...'
            self.publishing_pick.publish(self.msg)
            time.sleep(10)
            self.robot_plan[2], self.robot_plan[1] = True , False

        if self.robot_plan[2]:
            self.msg.data = 'Picking Action Completed'
            self.get_logger().info('Picking Operation Done')
            self.publishing_pick.publish(self.msg)
            time.sleep(1)
            self.get_logger().info('Placing .....')
            self.msg.data = 'Placing the object ...'
            self.publishing_place.publish(self.msg)
            time.sleep(10)
            self.robot_plan[3], self.robot_plan[2] = True , False

        if self.robot_plan[3]:
            self.msg.data = 'Placing Action Completed'
            self.get_logger().info('Placing Operation Done')
            self.publishing_place.publish(self.msg)
            time.sleep(1)
            self.robot_plan[3] = False


        goal_handle.succeed()
        
        result.response_data = 'All Movements Completed'
        return result


def main(args=None):
    rclpy.init(args=args)

    test_action_server = TestActionServer()

    rclpy.spin(test_action_server)


if __name__ == '__main__':
    main()