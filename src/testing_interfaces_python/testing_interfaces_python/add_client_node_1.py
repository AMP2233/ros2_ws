import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddThreeInts

class AdditionClientAsync1(Node):

    def __init__(self):
        super().__init__('add_client_node_1')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()

    def send_request(self, a, b, c):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        response = future.result()
        self.get_logger().info(
            'Result of add_three_ints: for %d + %d + %d = %d' %
            (self.req.a, self.req.b, self.req.c, response.sum))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    add_client_1 = AdditionClientAsync1()
    add_client_1.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    rclpy.spin(add_client_1)
    add_client_1.destroy_node()

if __name__ == '__main__':
    main()