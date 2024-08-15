# py_launch_example/scripts/twist_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.linear_increase = 0.005  # Incremento lineal por ciclo de temporizador
        self.linear_velocity = 0.0
        self.angular_velocity = 0.6  # Velocidad angular constante
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Publishing Twist messages')

    def timer_callback(self):
        msg = Twist()
        self.linear_velocity += self.linear_increase  # Incrementar la velocidad lineal gradualmente
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear={self.linear_velocity}, angular={self.angular_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
