## Project Architecture & In-Depth Tutorial

Below is a complete, line‑by‑line walk‑through of the **"Project‑Based Learning: Step‑by‑Step Recreation"** section. Each code snippet is followed by detailed explanations of **what each line does**, the **ROS2 concepts** it uses, and **why** we write it that way. Use this as a learning reference and compare against your own code.

---

### 1. Set Up Your ROS2 Workspace

```bash
# ① Create the workspace directory and a 'src' folder for source code
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# ② Build the workspace (even if empty) so colcon knows about it
colcon build --symlink-install
#    --symlink-install: allows editing source files without reinstalling

# ③ Source the installation overlay so ROS2 can find new packages
source install/setup.bash

# ④ Move into 'src' to create a package
cd src

# ⑤ Scaffold a new Python ROS2 package:
ros2 pkg create \
  --build-type ament_python \  # specifies Python-only build
  nav_controller \             # package name
  --dependencies rclpy nav_msgs geometry_msgs tf_transformations scipy numpy
#    rclpy: core ROS2 Python API
#    nav_msgs: messages for maps & odometry
#    geometry_msgs: Pose, Twist, etc.
#    tf_transformations: pose math
#    scipy, numpy: path planning & math

# ⑥ Return to workspace root and build only this package
cd ~/ros2_ws
colcon build --packages-select nav_controller
source install/setup.bash
# ⑦ Verify package exists on ROS2 package list
ros2 pkg list | grep nav_controller
```

**Key Concepts:**

* **colcon**: ROS2's build tool. Builds all packages in `src/`.
* **ament\_python**: Build type for Python packages; generates setup.py hooks.
* **ROS2 dependencies**: Declared in `package.xml` and installed automatically for imports.

---

### 2. Subscribe to Map and Odometry

```python
# control.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry

class NavDebug(Node):
    def __init__(self):
        # ① Initialize base Node with name 'nav_debug'
        super().__init__('nav_debug')

        # ② Create subscription to '/map' topic
        #    OccupancyGrid: map as a 2D grid of integers
        #    self.map_callback: callback on new messages
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # QoS history depth: keep last 10 messages
        )

        # ③ Subscribe to '/odom' for robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def map_callback(self, msg: OccupancyGrid):
        # ④ Extract width and height from map metadata
        width = msg.info.width
        height = msg.info.height
        # ⑤ Log info (info stream goes to console)
        self.get_logger().info(
            f"Map received: {width} x {height}"
        )

    def odom_callback(self, msg: Odometry):
        # ⑥ Read robot's position from Odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # ⑦ Log current pose
        self.get_logger().info(
            f"Robot at: x={x:.2f}, y={y:.2f}"
        )

def main(args=None):
    # ⑧ Initialize ROS2 Python client library
    rclpy.init(args=args)
    # ⑨ Instantiate our node
    node = NavDebug()
    # ⑩ Spin: block here to process incoming subscriptions
    rclpy.spin(node)
    # ⑪ Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Line‑by‑Line Concepts:**

1. **super().**init**('nav\_debug')**: Names the node; visible in `ros2 node list`.
2. **create\_subscription(...)**: Tells ROS2 to call our callback whenever a new message arrives.
3. **QoS depth=10**: Keeps at most 10 unprocessed messages; drop older ones.
4. **msg.info**: Metadata about the occupancy grid (resolution, origin, width, height).
5. **get\_logger().info(...)**: Standard way to print in ROS2 nodes (includes timestamps).
6. **msg.pose.pose.position**: Robot’s x, y, z in *map* or *odom* frame.
7. **Formatted strings**: `:.2f` limits to 2 decimal places.
8. **rclpy.init()**: Must be called before any ROS2 operations.
9. **node = NavDebug()**: Instantiates our class, sets up subscriptions.
10. **rclpy.spin(node)**: Enters a loop, keeps node alive to process callbacks.
11. **cleanup**: Gracefully shuts down the node and ROS client library.

---

### 3. Implement A\* Path Planner

```python
# astar.py
import heapq  # binary heap for priority queue
import numpy as np

def heuristic(a, b):
    """
    Manhattan distance between grid cells a,b.
    Suitable heuristic for 4‑connected grids.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from, start, goal):
    """
    Walk backwards from goal to start using came_from map.
    """
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def astar(grid: np.ndarray, start: tuple, goal: tuple):
    """
    grid: 2D numpy array; 0=free, 1=occupied
    start, goal: (row, col) tuples
    Returns: list of (row,col) path from start to goal, or [] if none
    """
    h, w = grid.shape
    # Open set: heap of (f_score, g_score, node, parent)
    open_set = []
    # Initialize with start node: f = heuristic(start,goal)
    heapq.heappush(open_set, (
        heuristic(start, goal),  # f_score = g+h, here g=0
        0,                        # g_score (cost from start)
        start,
        None                      # parent pointer
    ))
    came_from = {}     # maps node -> parent
    cost_so_far = {start: 0}  # g_score map

    while open_set:
        f, g, current, parent = heapq.heappop(open_set)
        if current == goal:
            # Found goal: record parent, reconstruct path
            came_from[current] = parent
            return reconstruct_path(came_from, start, goal)

        # Explore neighbors in 4 directions
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            # Check bounds and occupancy
            if (0 <= neighbor[0] < h and 0 <= neighbor[1] < w 
                and grid[neighbor] == 0):
                new_cost = g + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    # Push with priority f = g+h
                    heapq.heappush(open_set, (priority, new_cost, neighbor, current))
                    came_from[neighbor] = current
    # No path found
    return []
```

**Key Details:**

* **heapq**: efficient priority queue for selecting next best node.
* **g\_score**: cost from start; uniform +1 per step in grid.
* **heuristic**: admissible, never over‑estimates true cost.
* **came\_from**: dictionary to reconstruct final path.
* **4‑connected grid**: moves only up, down, left, right.

---

### 4. Costmap Inflation

```python
# costmap.py
import numpy as np

def inflate(grid: np.ndarray, radius: int) -> np.ndarray:
    """
    Mark all cells within 'radius' of any obstacle (value=1) as occupied (1).
    grid: binary array, 1=obstacle
    radius: number of cells to expand
    """
    h, w = grid.shape
    inflated = grid.copy()
    # For each obstacle cell, mark neighbors
    for i in range(h):
        for j in range(w):
            if grid[i, j] == 1:
                for di in range(-radius, radius+1):
                    for dj in range(-radius, radius+1):
                        ni, nj = i + di, j + dj
                        # Check in‑bounds
                        if 0 <= ni < h and 0 <= nj < w:
                            inflated[ni, nj] = 1
    return inflated
```

**Why Inflate?**

* Robots have non‑zero size and localization error. Inflating obstacles by a safety margin ensures the planned path keeps a safe distance.
* The double `for`‑loops over `di`/`dj` mark a square around each obstacle. A more advanced approach uses distance transforms, but this brute‑force method is fine for small maps.

---

### 5. B‑Spline Path Smoothing

```python
# bspline_planner.py
import numpy as np
from scipy.interpolate import splprep, splev

def smooth_path(path: list, num_points=200, degree=3) -> list:
    """
    Fit a B‑spline to the input path and sample it uniformly.
    path: list of (row,col) grid coords
    Returns: list of (row_f, col_f) floats in continuous space
    """
    # Extract x (col) and y (row) separately
    x = [p[1] for p in path]
    y = [p[0] for p in path]

    # splprep: parametric B‑spline representation
    # s=0: exact fit through points; k=degree
    tck, u = splprep([x, y], s=0, k=degree)

    # uniform parameter samples from 0 to 1
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)

    # Return list of (row, col)
    return list(zip(y_new, x_new))
```

**Technical Notes:**

* **splprep** outputs the spline representation (`tck`) and the parameterization `u` of input points.
* **splev** evaluates the spline at new parameter values.
* The result is a smooth, continuous path (floats), not restricted to grid centers.

---

### 6. Pure Pursuit Controller

```python
# pure_pursuit.py
import math

def pure_pursuit_control(
    path: list,
    pose: tuple,
    lookahead_dist=0.5,
    max_angular=1.0,
    max_linear=0.2
) -> tuple:
    """
    Given the robot's current pose and a path of waypoints,
    compute linear and angular velocities.
    pose: (x, y, yaw) in world frame
    path: list of (x_i, y_i) in world frame
    """
    x, y, yaw = pose

    # 1. Find the first path point at least lookahead_dist away
    for px, py in path:
        dist = math.hypot(px - x, py - y)
        if dist >= lookahead_dist:
            goal_x, goal_y = px, py
            break
    else:
        # If none found, use final point
        goal_x, goal_y = path[-1]

    # 2. Transform goal point into robot's local frame
    dx = goal_x - x
    dy = goal_y - y
    # rotation by -yaw:
    local_x = math.cos(-yaw)*dx - math.sin(-yaw)*dy
    local_y = math.sin(-yaw)*dx + math.cos(-yaw)*dy

    # 3. Steering: angle to lookahead point
    steering_angle = math.atan2(local_y, local_x)

    # 4. Compute angular velocity, clipped
    angular = max(-max_angular, min(max_angular, steering_angle))
    # 5. Linear velocity modulated by heading error
    linear = max_linear * (1 - abs(angular)/max_angular)

    return linear, angular
```

**Algorithm Breakdown:**

1. **Lookahead selection**: keeps a fixed preview distance to smooth control.
2. **Frame transform**: shift world coords into robot’s forward-ahead coordinate.
3. **Steering command**: aim toward lookahead point, yaw error = atan2.
4. **Velocity limits**: safety: max angular and linear speeds.

---

### 7. Integrate All in a ROS2 Node

```python
# control.py (full)
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
```

**Integration Points:**

1. **Subscriptions**: `/map` for world grid, `/goal_pose` set by RViz.
2. **Publisher**: `/cmd_vel` consumed by robot drivers.
3. **Callback storage**: track latest map & goal.
4. **Timer**: triggers control at fixed rate independent of callbacks.
5. **World ↔ Grid transforms**: use map origin & resolution.
6. **Placeholder methods**: must be replaced by actual `/odom` or TF lookup.

---

### 8. Launch & Visualization

```python
# launch/nav_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ① Spawn Gazebo world with TurtleBot3
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_world',
            name='gazebo'
        ),
        # ② Start SLAM for map building
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam'
        ),
        # ③ Our navigation controller
        Node(
            package='nav_controller',
            executable='control',  # from setup.py entry point
            name='nav_controller'
        ),
        # ④ RViz for interactive goal setting
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'path/to/config.rviz']
        ),
    ])
```

---

### 9. Node & Topic Architecture Tree

Below is a hierarchical view of all ROS2 nodes, their published and subscribed topics, and message types. Understanding this will help you see the data flow and how components interact.

```
└── nav_controller (Node)
    ├─ Subscriptions:
    │   ├─ /map           : nav_msgs/OccupancyGrid
    │   └─ /goal_pose     : geometry_msgs/PoseStamped
    ├─ Publications:
    │   └─ /cmd_vel       : geometry_msgs/Twist
    └─ Timers:
        └─ control_loop (20 Hz)

└── turtlebot3_bringup (Node)
    ├─ Subscriptions:
    │   └─ /cmd_vel       : geometry_msgs/Twist
    ├─ Publications:
    │   ├─ /odom          : nav_msgs/Odometry
    │   └─ /tf            : tf2_msgs/TFMessage

└── slam_toolbox (Node)
    ├─ Subscriptions:
    │   ├─ /scan          : sensor_msgs/LaserScan
    │   └─ /tf            : tf2_msgs/TFMessage
    ├─ Publications:
    │   ├─ /map           : nav_msgs/OccupancyGrid
    │   └─ /tf            : tf2_msgs/TFMessage

└── rviz2 (Node)
    ├─ Subscriptions:
    │   ├─ /map           : nav_msgs/OccupancyGrid
    │   ├─ /odom          : nav_msgs/Odometry
    │   ├─ /tf            : tf2_msgs/TFMessage
    │   └─ /cmd_vel       : geometry_msgs/Twist (for arrow)
    └─ Publications:
        └─ /goal_pose     : geometry_msgs/PoseStamped
```

#### Explanation of Each Node & Topic

1. **nav\_controller**

   * **Subscriptions:**

     * `/map` (nav\_msgs/OccupancyGrid)

       * *Mechanism:* Receives the latest 2D grid map of the environment from SLAM.
       * *Use:* Converts to binary grid for planning and obstacle inflation.
     * `/goal_pose` (geometry\_msgs/PoseStamped)

       * *Mechanism:* Gets the target pose set by the user in RViz.
       * *Use:* Triggers A\*, B‑Spline smoothing, and path storage.
   * **Publications:**

     * `/cmd_vel` (geometry\_msgs/Twist)

       * *Mechanism:* Sends velocity commands to the robot base controller.
       * *Use:* Drives the robot along the planned path via Pure Pursuit.
   * **Timers:**

     * `control_loop` @ 20 Hz

       * *Mechanism:* Calls pure\_pursuit\_control at fixed rate.
       * *Use:* Ensures smooth, real‑time feedback control.

2. **turtlebot3\_bringup**

   * **Subscriptions:**

     * `/cmd_vel` (geometry\_msgs/Twist)

       * *Mechanism:* Robot’s low‑level controller listens for velocity commands.
       * *Use:* Converts Twist to motor commands.
   * **Publications:**

     * `/odom` (nav\_msgs/Odometry)

       * *Mechanism:* Publishes wheel‑encoder–based pose estimate.
     * `/tf` (tf2\_msgs/TFMessage)

       * *Mechanism:* Broadcasts transform from `odom` frame to `base_link`.

3. **slam\_toolbox**

   * **Subscriptions:**

     * `/scan` (sensor\_msgs/LaserScan)

       * *Mechanism:* Laser range data from robot’s LiDAR.
     * `/tf` (tf2\_msgs/TFMessage)

       * *Mechanism:* Receives current robot pose to update map.
   * **Publications:**

     * `/map` (nav\_msgs/OccupancyGrid)

       * *Mechanism:* Builds and updates the 2D occupancy grid online.
     * `/tf` (tf2\_msgs/TFMessage)

       * *Mechanism:* Provides refined pose transforms for localization.

4. **rviz2**

   * **Subscriptions:**

     * `/map`, `/odom`, `/tf`, `/cmd_vel`

       * *Mechanism:* Visualizes environment, robot pose, path, and motion.
   * **Publications:**

     * `/goal_pose` (geometry\_msgs/PoseStamped)

       * *Mechanism:* User clicks in RViz to send a goal to `nav_controller`.

---

## How the System Works Together

1. **Mapping & Localization**

   * `turtlebot3_bringup` starts the robot driver, publishing `/odom` & `/tf`.
   * `slam_toolbox` fuses LiDAR (`/scan`) & odometry (`/tf`) to publish a consistent `/map` and refined `/tf`.

2. **User Input**

   * In RViz, the user selects a 2D Nav Goal, which publishes a `PoseStamped` on `/goal_pose`.

3. **Path Planning**

   * `nav_controller` receives `/map` & `/goal_pose`:

     * Inflates obstacles in the occupancy grid.
     * Runs A\* to get a raw grid path.
     * Smooths the path with a B‑Spline.

4. **Path Tracking**

   * The `control_loop` timer reads the current robot pose (via `/odom` & `/tf`).
   * Pure Pursuit picks a lookahead point, computes linear/angular velocities.
   * Publishes `Twist` on `/cmd_vel`.

5. **Robot Motion**

   * `turtlebot3_bringup` receives `/cmd_vel`, controls motors.
   * Robot moves; new `/odom` & `/tf` feedback updates SLAM and control.

By tracing each topic and node, you can see the closed loop: **sensor → SLAM → map → planner → controller → actuators → sensors**. This modular architecture lets you swap planners, controllers, or sensors independently.

---

## Final Notes for Juniors

* Use `ros2 topic echo` and `ros2 topic list` to explore messages.
* Try injecting delays or noise to see robustness.
* Experiment with lookahead distances or A\* heuristics.

Happy learning and coding!
