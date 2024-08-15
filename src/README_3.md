# Practica N° 3 - Lauch files
Para esta practica se nos pidio que implementemos los archivos launch tanto en **C++** y **Python**, para mover la `turtlesim1` mediante el `mimic` para que la `turtlesim2` siga el trayecto de manera sincronizada.
## Launch file - Python
### Practica Python
Para los launch files del Python se va ir modificando en el archivo `turtlesim_mimic_launch.py` para esta primera parte se va modificar lo codigo de este archivo:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            parameters=[
                {"background_r": 0},
                {"background_g": 0},
                {"background_b": 0}
            ]
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            output='screen',
            remappings=[
                ('/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
            ]
        )
    ])
```
En el nodo de `turtlesim1` se puso el fondo negro para diferenciar con el `turtlesim2`
```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            parameters=[
                {"background_r": 0},
                {"background_g": 0},
                {"background_b": 0}
            ]
        )
```
Adems que se implementa un nodo para implementar `turtle_teleop_key` para poder manejar `turtlesim1`:
```python
Node(
    package='turtlesim',
    executable='turtle_teleop_key',
    name='teleop',
    output='screen',
    remappings=[
        ('/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
    ]
)
```
Para implementar el funcionamiento de esta practica se debe ejcutar el siguiente comando en la terminal:
```
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtlesim1/turtle1/cmd_vel
```
Este comando se debe ejecutar cuando queramos poner `turtle_teleop_key` y podamos mover la `turtlesim1`.
Para ejecutar el archivo launch de la practica se debe ejecutar con el este comando:
```
ros2 launch py_launch_example turtlesim_mimic_launch.py
```
El `turtle_teleop_key` se va subcribir a `turtlesim1` generando su respectivo movieminto al ejecutar el comando, `turtlesim2` lo va tratar de seguir.
### Opcional Python
Para la parte opcional se ha puesto un archivo launch para la implementacion se puso un nodo donde se pone el archivo `twist_publisher.py` que el codigo es el siguiente:
```python
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
```
Y el nodo ser veria asi:
```python
Node(
    package='py_launch_example',
    executable='twist_publisher',
    name='twist_publisher',
    output='screen'
    )
```
Hay que ejecutar el `turtlesim_mimic_b_launch.py` mediante este comando para que dibuje la espiral:
```
ros2 launch py_launch_example turtlesim_mimic_b_launch.py
```
## Launch file - C++
### Practica C++
Para esta practica en `C++` se hace lo mismo y se ejecuta sl mismo comando :
```
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtlesim1/turtle1/cmd_vel
```
y para que ejecute el comando el launch:
```
ros2 launch cpp_launch_example turtlesim_mimic_launch.py
```
### Opcional C++
En esta seccion se implementa el codigo de `twist_publisher.cpp` que esta en `C++`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher()
    : Node("twist_publisher"), linear_velocity_(0.0), angular_velocity_(0.5)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtlesim1/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TwistPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing Twist messages");
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        linear_velocity_ += 0.005;  // Incremento lineal por ciclo de temporizador
        msg.linear.x = linear_velocity_;
        msg.angular.z = angular_velocity_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.3f, angular=%.3f", linear_velocity_, angular_velocity_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_velocity_;
    const double angular_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
En la parte del opcional, solo hay que ejecutar el siguiente comando:
```
ros2 launch cpp_launch_example turtlesim_mimic_b_launch.py
```
## Preguntas
### ¿Por qué una de las tortugas no sigue exactamente a la otra?
    1. Frecuencia de Publicación: La frecuencia con la que se publica y procesa la información puede no ser lo suficientemente alta. Si la frecuencia de publicación de los mensajes de control (cmd_vel) no coincide con la frecuencia con la que los nodos están actualizando sus posiciones, podría haber un desfase en el movimiento. 
    2. Diferencias en la Implementación de los Nodos: Los nodos turtlesim y mimic pueden tener diferencias en la forma en que procesan los datos de entrada y generan los movimientos. Por ejemplo, la velocidad angular y lineal podría no ser replicada con precisión debido a diferencias en la interpretación o aplicación de los comandos de velocidad.
    3. Limitaciones del Nodo Mimic: El nodo mimic simplemente replica los comandos de velocidad (cmd_vel) que recibe, pero la precisión con la que esto se lleva a cabo depende de cómo el nodo maneja las pequeñas variaciones en los datos de entrada. Puede que el nodo no sea capaz de replicar todos los detalles finos del movimiento de la tortuga líder.
    4. Resolución de Simulación: Las simulaciones en turtlesim tienen limitaciones en términos de resolución y precisión, lo que puede llevar a ligeras diferencias en el movimiento.

## Explique el enfoque a seguir para resolver el problema de que una tortuga no sigue exactamente a la otra.

    Para resolver el problema de que una tortuga no sigue exactamente a la otra en el entorno de simulación de ROS 2 con `turtlesim`, se puede seguir un enfoque estructurado que aborde los posibles factores que causan este comportamiento. Aquí hay un enfoque detallado:

         1. Ajuste de la Frecuencia de Publicación y Suscripción
        - Verificar la Frecuencia de Publicación: Asegúrate de que la frecuencia con la que se publican los comandos de velocidad (`cmd_vel`) desde `turtle_teleop_key` es alta y consistente. Esto se puede lograr utilizando el argumento `-r` en el comando de `ros2 topic pub` para establecer una frecuencia de publicación adecuada.

        - Sincronización de la Frecuencia de los Nodos: Asegúrate de que la frecuencia de publicación del nodo `mimic` coincide con la frecuencia a la que se generan los comandos de la tortuga líder. Esto minimizará el desfase en el seguimiento.

        2. Optimización del Nodo Mimic
        - Revisión del Nodo Mimic: Analiza y, si es necesario, modifica el nodo `mimic` para mejorar cómo replica los comandos de velocidad. Asegúrate de que este nodo maneja de manera eficiente y precisa los comandos recibidos para minimizar el retraso o pérdida de información.
        - Mejorar la Interpolación de Movimiento: Si el nodo `mimic` utiliza alguna forma de interpolación para generar movimientos suaves, asegúrate de que este proceso no esté introduciendo errores que causen que la tortuga seguidora se desvíe.

         3. Reducir la Latencia de Comunicación
        - Verificar la Red y el Hardware: Si se está ejecutando en un entorno distribuido o en un hardware con limitaciones, considera optimizar la red o utilizar un hardware más adecuado para reducir la latencia.
        - Uso de QoS (Quality of Service): Configura las políticas de QoS en ROS 2 para asegurar que los mensajes sean entregados con la menor latencia posible. Por ejemplo, podrías usar configuraciones de QoS que prioricen la baja latencia y la entrega confiable de mensajes.

        4. Simulación de Prueba y Ajuste
        - Ajustes Experimentales: Realiza pruebas ajustando gradualmente los parámetros anteriores y observa cómo afecta el seguimiento de la tortuga. Documenta los resultados para identificar el ajuste óptimo.
         - Monitoreo y Debugging: Utiliza herramientas de monitoreo y depuración de ROS 2 (como `rqt_graph`, `ros2 topic echo`, y `ros2 topic hz`) para verificar que los datos están siendo transmitidos y procesados en tiempo real de manera eficiente.

         5. Evaluación de Alternativas
         - Alternativas al Nodo Mimic: Si las modificaciones anteriores no logran resolver el problema, considera crear un nodo personalizado que maneje el seguimiento de la tortuga seguidora de manera más precisa, posiblemente implementando un control cerrado basado en la diferencia de posición o velocidad entre las dos tortugas.

        6. Validación Final
         - Comparación de Resultados: Después de aplicar las soluciones, compara los movimientos de ambas tortugas para asegurarte de que la tortuga seguidora replica fielmente el movimiento de la tortuga líder. 
        - Documentación: Documenta el proceso y los cambios realizados para referencia futura y para mejorar la solución si el problema vuelve a ocurrir.
## ¿Los parámetros de QoS tienen algo que ver con el problema de que una tortuga no sigue exactamente a la otra? ¿Cómo interfieren estos en este comportamiento?
Sí, los parámetros de Calidad de Servicio (QoS) en ROS 2 pueden influir significativamente en la precisión con la que una tortuga sigue a la otra en el entorno de turtlesim.

### Impacto de los Parámetros de QoS:
1. **Durabilidad (Durability)**: Si la durabilidad está configurada como "transient local", los mensajes anteriores pueden ser retenidos, lo que podría causar un desfase si la segunda tortuga recibe datos atrasados. Usar durabilidad "volatile" asegura que solo se reciben mensajes nuevos.

2. **Fiabilidad (Reliability)**: La fiabilidad "best effort" envía mensajes sin garantizar que lleguen todos, lo que puede causar que la segunda tortuga omita algunos comandos de movimiento. "Reliable" garantiza la entrega de todos los mensajes, pero podría introducir latencia si hay problemas en la red.

3. **Historial (History)**: El historial "keep last" puede limitar el número de mensajes almacenados, potencialmente descartando mensajes que aún no han sido procesados por la segunda tortuga. Configurar un historial con un tamaño mayor podría mejorar la precisión, pero a costa de mayor consumo de memoria.

4. **Profundidad del Historial (History Depth)**: Un valor bajo podría resultar en pérdida de datos si los mensajes se generan más rápido de lo que pueden ser procesados.
## En la primera parte de la práctica (parte a), al presionar la tecla de flecha hacia abajo, la segunda tortuga sigue avanzando, ¿por qué sucede esto? ¿Cómo solucionar este comportamiento?
Este comportamiento sucede porque la configuración de mapeo de las velocidades podría estar afectando el movimiento de la segunda tortuga (turtlesim2). Si la segunda tortuga no puede recibir comandos de velocidad negativos para moverse hacia atrás, es posible que siga avanzando debido a que solo recibe comandos positivos o que no se esté enviando correctamente el comando para moverse en dirección contraria.

### Enfoque para resolver el problema:

1. **Verificar los Comandos de Velocidad**: Asegúrate de que el comando `/turtlesim2/turtle1/cmd_vel` esté recibiendo correctamente los valores negativos para la velocidad lineal en el eje `x`. Puedes probar manualmente enviando un comando de velocidad negativa para verificar si la tortuga responde.

2. **Revisar la Remapping**: Asegúrate de que los remappings entre `/turtlesim1/turtle1/cmd_vel` y `/turtlesim2/turtle1/cmd_vel` están configurados correctamente y no hay ninguna lógica que impida enviar comandos negativos.

3. **QoS Parameters**: Verifica los parámetros de QoS (Quality of Service). Asegúrate de que no haya configuraciones que estén limitando la frecuencia o la confiabilidad de los mensajes entre las dos tortugas, lo que podría causar que los comandos no se reciban adecuadamente.

4. **Debugging con Logs**: Puedes agregar logs en el código o usar `ros2 topic echo` para ver los valores exactos que se están enviando a la segunda tortuga cuando presionas la tecla de flecha hacia abajo.

Implementando estos pasos debería resolver el problema, permitiendo que la segunda tortuga pueda moverse hacia atrás correctamente cuando se presione la tecla correspondiente.