import time
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from custom_actions.action import RotateJoint

class ActionServerClass(Node):
    def __init__(self):
        super().__init__('joint_rotation_action_server')
        self.server=ActionServer(self, RotateJoint, 'joint_rotation', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        target_angle=goal_handle.request.target_angle #extract the goal request

        result = RotateJoint.Result()
        fdbk = RotateJoint.Feedback()

        self.get_logger().info(f"Goal Request: {target_angle}")
        
        #accept or reject the goal based on some criteria
        if target_angle <=10.0 and target_angle >= -10.0:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            result.success=False

        fdbk.current_angle = 0.0
        while True:
            self.get_logger().info(f"Current angle: {fdbk.current_angle}")
            goal_handle.publish_feedback(fdbk)
            if target_angle==fdbk.current_angle:
                result.success=True
                break
            fdbk.current_angle+=1.0
            time.sleep(1.0)

        return result
    
def main():
    rclpy.init()
    node=ActionServerClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()