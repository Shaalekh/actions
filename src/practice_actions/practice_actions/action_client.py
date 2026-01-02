import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from custom_actions.action import RotateJoint

class ActionClientClass(Node):
    def __init__(self):
        super().__init__('joint_rotation_action_client')
        self._action_client: ActionClient = ActionClient(self, RotateJoint, 'joint_rotation')

    def send_goal(self, angle: float) -> None:
        payload = RotateJoint.Goal()
        payload.target_angle = angle
        
        # Wait for server with timeout and feedback
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return
        
        self.get_logger().info(f"Sending goal: rotate to {angle}")
        
        # Send goal asynchronously
        self.send_goal_future = self._action_client.send_goal_async(
            payload, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future) -> None:
        self.get_logger().debug('Inside goal response callback')
        goal_handle: ClientGoalHandle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info("Goal Rejected")
            return
        else:
            self.get_logger().info('Goal Accepted')

        # After the acknowledgement of goal receipt, get the result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future) -> None:
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current angle: {feedback.current_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = ActionClientClass()
    node.send_goal(5.0)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
