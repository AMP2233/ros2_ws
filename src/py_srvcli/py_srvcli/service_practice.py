import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class ServicePractice(Node):

    def __init__(self):
        super().__init__('service_practice')
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        
        self.subscription
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.current_color = None

    def pose_callback(self, msg):
        if msg.x > 5.5:
            if self.current_color != 'red':
                self.change_pen_color(255, 0, 0)
                self.current_color = 'red'
                self.get_logger().info('Set pen color to red')
        else:
            if self.current_color != 'green':
                self.change_pen_color(0, 255, 0)
                self.current_color = 'green'
                self.get_logger().info('Set pen color to green')

    def change_pen_color(self, r, g, b):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 3
        request.off = 0
        self.pen_client.call_async(request)
        
def main():
    rclpy.init()
    service_practice_node = ServicePractice()
    rclpy.spin(service_practice_node)
    service_practice_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Poner el nombre send_request genera que no se ponga en el punto que deseamos
# poner en self.request = SetPen.Request() debajo en la linea 15 genera que este se llame al servicio constantemente