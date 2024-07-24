import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NodeSubscriberClient(Node):

   def __init__(self):
      super().__init__('client_subscription_node_fail')
      self.subscription_ = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
      self.subscription_  # prevent unused variable warning
      self.cli = self.create_client(AddTwoInts, 'add_two_ints')
      while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
      self.req = AddTwoInts.Request()

   def send_request(self, a, b):
      self.req.a = a
      self.req.b = b
      self.future = self.cli.call_async(self.req)
      # rclpy.spin_until_future_complete(self, self.future)
      # return self.future.result()
      return self.future

   def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)
      if (msg.data == "Hello World: 10"):
            self.get_logger().info('Calling add_two_ints the service...')
            future = self.send_request(2,5)
            future.add_done_callback(self.callback_sum)

   def callback_sum(self, future):
      if future.result() is not None:
            res = future.result()
            self.get_logger().info('The sum is: "%s"' % res.sum)
      else:
            self.get_logger().warning('Service call failed')

def main():
   rclpy.init()

   sub_client_node = NodeSubscriberClient()
   rclpy.spin(sub_client_node)
   sub_client_node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()