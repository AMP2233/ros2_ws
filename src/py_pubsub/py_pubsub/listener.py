import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

   def __init__(self):
      super().__init__('listener') #Se va crear un nodo de nombre "listener"
      self.subscription_ = self.create_subscription(
            String, #El tipo de mensaje que va enviar
            'topic', #El topic al cual va estar suscrito
            self.listener_callback, #Funcion que se va bejecutar cada que se envie este mensaje
            10)
      self.subscription_  # prevent unused variable warning

   def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data) #Aqui indica que mensaje se va mostrar en la terminal


def main(args=None):
   rclpy.init(args=args)

   minimal_subscriber = MinimalSubscriber()

   rclpy.spin(minimal_subscriber)

   minimal_subscriber.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()