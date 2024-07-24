import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen 

class ServicePracticeB(Node):

    def __init__(self):
        super().__init__('service_practice_b')
        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.pen_service_client = self.create_client(SetPen, 'turtle1/set_pen')  # Cambié a SetPen
        self.pose = Pose()
        self.is_near_wall = False
        self.color = 'green'

    def pose_callback(self, msg):
        self.pose = msg
        self.check_boundaries()
        self.update_pen_color()
        self.move_turtle()

    def check_boundaries(self):
        if self.pose.x < 1.0 or self.pose.x > 10.0 or self.pose.y < 1.0 or self.pose.y > 10.0:
            self.is_near_wall = True
        else:
            self.is_near_wall = False

    def update_pen_color(self):
        if self.pose.x > 5.5 and self.color != 'red':
            self.set_pen_color(255, 0, 0)  # Rojo
            self.color = 'red'
            self.get_logger().info('Set pen color to red')
        elif self.pose.x <= 5.5 and self.color != 'green':
            self.set_pen_color(0, 255, 0)  # Verde
            self.color = 'green'
            self.get_logger().info('Set pen color to green')

    def set_pen_color(self, r, g, b):
        if self.pen_service_client.wait_for_service(timeout_sec=1.0):
            req = SetPen.Request()
            req.r = r
            req.g = g
            req.b = b
            req.width = 3  # Ancho del bolígrafo
            req.off = 0  # Boligrafo encendido
            self.pen_service_client.call_async(req)
        else:
            self.get_logger().error('Service not available')

    def move_turtle(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.9 if self.is_near_wall else 1.0
        vel_msg.angular.z = 0.9 if self.is_near_wall else 0.0
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    service_practice_b_node = ServicePracticeB()
    rclpy.spin(service_practice_b_node)
    service_practice_b_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
