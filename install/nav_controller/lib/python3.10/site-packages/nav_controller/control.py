# nav_controller/control.py
import rclpy
from .navigation_node import NavigationControl

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()