import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil

class MemoryPublisher(Node):
    def __init__(self):
        super().__init__('mempub')
        self.publisher_ = self.create_publisher(String, 'memory_usage', 10)
        self.timer = self.create_timer(1.0, self.publish_memory_info)  # Publica a cada 1 segundo

    def publish_memory_info(self):
        memory = psutil.virtual_memory()
        total_memory = memory.total / (1024 ** 3)  # Converte para GB
        used_memory = memory.used / (1024 ** 3)  # Converte para GB
        percent_usage = memory.percent

        message = f"Total: {total_memory:.2f} GB, Used: {used_memory:.2f} GB, Usage: {percent_usage:.2f}%"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    memory_publisher = MemoryPublisher()
    rclpy.spin(memory_publisher)
    memory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
