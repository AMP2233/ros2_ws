import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TopicsPractice(Node):

    def __init__(self):
        super().__init__('topics_practice')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)  # Modificación: Ajuste del temporizador
        self.countdown = 5
        self.in_spiral_mode = True
        self.linear_velocity = 0.9
        self.angular_velocity = 1.1  # Modificación: Ajuste de la velocidad angular
        self.current_x = 0.0
        self.current_y = 0.0

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y

    def timer_callback(self):
        if self.countdown > 0:
            self.get_logger().info(f'Application starts in: {self.countdown}')
            self.countdown -= 1
        else:
            if self.in_spiral_mode:
                self.draw_spiral()
            else:
                self.go_straight()

    def draw_spiral(self):
        msg = Twist()
        self.linear_velocity += 0.08  # Modificación: Incremento de la velocidad lineal para hacer crecer la espiral
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Drawing spiral')
        if self.reached_boundaries():
            self.in_spiral_mode = False

    def go_straight(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Going straight')

    def reached_boundaries(self):
        # Modificación: Ajuste de los límites de la pantalla
        x_limit = 9.0
        y_limit = 9.0
        if self.current_x > x_limit or self.current_x < 1.0 or self.current_y > y_limit or self.current_y < 1.0:
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    topics_practice = TopicsPractice()
    rclpy.spin(topics_practice)
    topics_practice.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()