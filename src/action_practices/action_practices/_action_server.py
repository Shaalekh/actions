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
        sequence = [0,1]
        for i in range (1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

        goal_handle.succeed()
        result=Fibonacci.Result()
        result.sequence=sequence
        return result
    

def main(args=None):
    rclpy.init(args=args)
    sode=FibonacciServerClass()
    rclpy.spin(sode)
    sode.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()