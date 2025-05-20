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