import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciServerClass(Node):
    def __init__(self):
        super().__init__('action_server')
        self._action_server=ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self._execute_callback)
        
    def _execute_callback(self, goal_handle):
        self.get_logger().info('Eecuting goal...')
        
        fdb_msg=Fibonacci.Feedback()
        fdb_msg.sequence = [0,1]

        for i in range (1, goal_handle.request.order):
            fdb_msg.sequence.append(fdb_msg.sequence[i] + fdb_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(fdb_msg.sequence))
            goal_handle.publish_feedback(fdb_msg)
            time.sleep(1.0)

        goal_handle.succeed()
        result=Fibonacci.Result()
        result.sequence=fdb_msg.sequence
        return result
    

def main(args=None):
    rclpy.init(args=args)
    sode=FibonacciServerClass()
    rclpy.spin(sode)
    sode.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()