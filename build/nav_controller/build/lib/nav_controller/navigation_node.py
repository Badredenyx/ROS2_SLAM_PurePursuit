# nav_controller/navigation_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np                                            # ← fix (1)

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist

from .planner import costmap, astar
from .smoother import bspline_planning
from .pure_pursuit import pure_pursuit
from .utils import euler_from_quaternion

class NavigationControl(Node):
    def __init__(self):
        super().__init__('navigation_control')
        # Subscriptions
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_cb,
            QoSProfile(depth=10)
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer at 100 Hz
        self.timer = self.create_timer(0.01, self.timer_cb)

        # Initialize pose & path variables (fix (2))
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.path = []
        self.idx = 0

        # State flag: 0 = waiting for goal, 1 = got goal, 2 = following path
        self.flag = 0

        print("Waiting for goal...")

    def goal_cb(self, msg):
        # Called when a new goal is set via RViz (PoseStamped)
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        print(f"Goal set: {self.goal}")
        self.flag = 1

    def map_cb(self, msg):
        # Only run when a goal has just been received (flag == 1)
        if self.flag != 1:
            return

        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        # Convert robot pose into grid indices
        start = (int((self.y - oy) / res), int((self.x - ox) / res))
        end = (int((self.goal[1] - oy) / res), int((self.goal[0] - ox) / res))

        # Build costmap and inflate obstacles
        grid = costmap(msg.data, msg.info.width, msg.info.height, res)

        # Guarantee start-cell is free
        grid[start[0], start[1]] = 0

        # Mark “unknown” (< 0) or “inflated wall” (>= 5.0) as obstacle = 1, else free = 0 (fix (3))
        grid = np.where(grid < 0, 1, np.where(grid >= 5.0, 1, 0))

        # Run A*
        raw_path = astar(grid, start, end)

        # Convert from grid indices back to world coordinates
        path_world = [
            (p[1] * res + ox, p[0] * res + oy)
            for p in raw_path
        ]

        # Smooth the path with B-splines (optional)
        self.path = bspline_planning(path_world)

        print(f"Path generated. Robot at: ({self.x:.2f}, {self.y:.2f})")
        self.flag = 2  # Now enter “pursuit” mode

    def odom_cb(self, msg):
        # Always update current pose from Odometry
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

    def timer_cb(self):
        # Only run Pure Pursuit once a path exists (flag == 2)
        if self.flag != 2:
            return

        # Get a velocity & steering angle for current pose
        v, steer, self.idx = pure_pursuit(
            self.x, self.y, self.yaw,
            self.path, self.idx
        )

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = steer

        # If we’re within 5 cm of the final path point, stop and reset
        if (
            abs(self.x - self.path[-1][0]) < 0.05 and
            abs(self.y - self.path[-1][1]) < 0.05
        ):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print("Reached goal. Waiting for new goal...")
            self.flag = 0  # go back to “waiting” state

        self.cmd_pub.publish(twist)
