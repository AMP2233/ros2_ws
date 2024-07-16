import rclpy #LLibreia de ROS2
from rclpy.node import Node #Importamos de la clase nodo del modulo rclpy.node, nos ayuda a crear un nodo
from std_msgs.msg import String #Importamos mensaje de tipo string del modulo std_msgs.msg 


class MinimalPublisher(Node):

   def __init__(self):
      super().__init__('publisher')#Nombre del nodo
      self.publisher_ = self.create_publisher(String, 'topic', 10)# Variable propia del clase MinimalPublisher
      #El create_publisher define un objeto que publca mensajes de tipo string
      #por un topic llamado topic con un tamaño de cola de 10
      timer_period = 0.5  # seconds
      # Crea el temporizador para el mensaje
      self.timer_ = self.create_timer(timer_period, self.timer_callback)
      self.count_ = 0

   def timer_callback(self):
      msg = String() # Mensaje de tipo string
      msg.data = 'Hello World: %d' % self.count_ # El mensaje va ser un Hello World seguidas de un conteo
      self.publisher_.publish(msg) # Publica el mensaje
      self.get_logger().info('Publishing: "%s"' % msg.data) # Muestra el mensaje que hemos creado
      self.count_ += 1


def main(args=None):
   rclpy.init(args=args)

   minimal_publisher = MinimalPublisher() # Al crear este objeto se crea el nodo de nombre "Publisher"

   rclpy.spin(minimal_publisher) # Todos que estan detro del ambiente de ROS se van a ejecutar para el nodo "minimal_publsiher" que se paso por parametros

   minimal_publisher.destroy_node()# Va destruir el nodo cuando paremos desde la terminal
   rclpy.shutdown()


if __name__ == '__main__':
   main()