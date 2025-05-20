# nav_controller/navigation_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from .planner import costmap, astar
from .smoother import bspline_planning
from .pure_pursuit import pure_pursuit
from .utils import euler_from_quaternion

class NavigationControl(Node):
    def __init__(self):
        super().__init__('navigation_control')
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, QoSProfile(depth=10))
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.flag = 0
        self.idx = 0
        print("Waiting for goal...")

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        print(f"Goal set: {self.goal}")
        self.flag = 1

    def map_cb(self, msg):
        if self.flag != 1:
            return
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        start = (int((self.y-oy)/res), int((self.x-ox)/res))
        end = (int((self.goal[1]-oy)/res), int((self.goal[0]-ox)/res))
        grid = costmap(msg.data, msg.info.width, msg.info.height, res)
        grid[start[0], start[1]] = 0
        grid = np.where(grid<0,1, np.where(grid>5,1,0))
        raw_path = astar(grid, start, end)
        path_world = [(p[1]*res+ox, p[0]*res+oy) for p in raw_path]
        self.path = bspline_planning(path_world)
        print(f"Path generated. Robot at: ({self.x:.2f},{self.y:.2f})")
        self.flag = 2

    def odom_cb(self, msg):
        pose = msg.pose.pose
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = euler_from_quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)

    def timer_cb(self):
        if self.flag != 2:
            return
        v, steer, self.idx = pure_pursuit(self.x, self.y, self.yaw, self.path, self.idx)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = steer
        if abs(self.x - self.path[-1][0])<0.05 and abs(self.y - self.path[-1][1])<0.05:
            twist.linear.x = twist.angular.z = 0.0
            print("Reached goal. Waiting for new goal...")
            self.flag = 0
        self.cmd_pub.publish(twist)