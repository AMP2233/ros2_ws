import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TopicsPracticeB(Node):

    def __init__(self):
        super().__init__('topics_practice_b')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.countdown = 5
        self.in_spiral_mode = True
        self.linear_velocity = 0.9
        self.angular_velocity = 1.1
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
                self.go_straight_or_avoid_walls()

    def draw_spiral(self):
        msg = Twist()
        self.linear_velocity += 0.08
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Drawing spiral')
        if self.reached_boundaries():
            self.in_spiral_mode = False

    def go_straight_or_avoid_walls(self):
        if self.near_wall():
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.publisher_.publish(msg)
            self.get_logger().info('Avoiding walls')
        else:
            msg = Twist()
            msg.linear.x = self.linear_velocity
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Going straight')

    def near_wall(self):
        x_limit = 1.0
        y_limit = 1.0
        if self.current_x < x_limit or self.current_x > 10.5 - x_limit or self.current_y < y_limit or self.current_y > 10.5 - y_limit:
            return True
        return False

    def reached_boundaries(self):
        x_limit = 9.0
        y_limit = 9.0
        if self.current_x > x_limit or self.current_x < 1.0 or self.current_y > y_limit or self.current_y < 1.0:
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    # Run topics_practice_b node
    topics_practice_b = TopicsPracticeB()
    rclpy.spin(topics_practice_b)
    topics_practice_b.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()