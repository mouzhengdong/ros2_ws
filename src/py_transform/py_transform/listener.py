import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TF2Subscriber(Node):
    def __init__(self):
        super().__init__('tf2_subscriber')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.lookup_transform)  # 每0.1秒查看一次变换
    
    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"Transform: {transform.transform.translation.x}")
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Could not lookup transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TF2Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()