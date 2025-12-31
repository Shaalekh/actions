import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciClientClass(Node):
    def __init__(self):
        super().__init__('action_client')
        self._action_client=ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    
def main():
    rclpy.init()
    code=FibonacciClientClass()
    future=code.send_goal(10)
    rclpy.spin_until_future_complete(code, future)
    code.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()