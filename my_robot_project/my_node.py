import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.create_timer(0.1, self.drive)
        self.get_logger().info('Robot node started!')

    def drive(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
