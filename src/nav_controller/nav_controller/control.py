import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav_controller.astar import astar
from nav_controller.costmap import inflate
from nav_controller.bspline_planner import smooth_path
from nav_controller.pure_pursuit import pure_pursuit_control

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')

        # ① Subscribe to map and goal topics
        self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        # ② Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ③ State storage
        self.map = None
        self.map_info = None
        self.smoothed = []

        # ④ Timer: control loop at 20Hz
        self.create_timer(0.05, self.control_loop)

    def map_cb(self, msg: OccupancyGrid):
        # ⑤ Store map and metadata
        self.map_info = msg.info
        data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        # Binary occupancy: >50 means obstacle
        self.map = (data > 50).astype(int)

    def goal_cb(self, msg: PoseStamped):
        # ⑥ Convert goal to grid coordinates
        gx = int((msg.pose.position.y -
                  self.map_info.origin.position.y)
                 / self.map_info.resolution)
        gy = int((msg.pose.position.x -
                  self.map_info.origin.position.x)
                 / self.map_info.resolution)
        goal_cell = (gx, gy)

        # ⑦ Inflate and plan raw path
        inflated = inflate(self.map, radius=5)
        start_cell = self.get_robot_cell()
        raw_path = astar(inflated, start_cell, goal_cell)

        # ⑧ Smooth path and convert back to world coords
        smooth = smooth_path(raw_path)
        self.smoothed = [(
            y * self.map_info.resolution + self.map_info.origin.position.y,
            x * self.map_info.resolution + self.map_info.origin.position.x
        ) for (y, x) in smooth]

    def get_robot_cell(self):
        # ⑨ Placeholder: normally convert /odom or TF to grid cell
        # For demo, return fixed start
        return (0, 0)

    def control_loop(self):
        # ⑩ Only run if we have a path
        if not self.smoothed:
            return

        # ⑪ Read current pose (placeholder)
        pose = (0.0, 0.0, 0.0)

        # ⑫ Compute control
        lin, ang = pure_pursuit_control(self.smoothed, pose)

        # ⑬ Publish command velocities
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()