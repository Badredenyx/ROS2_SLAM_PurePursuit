# nav_controller/pure_pursuit.py
import math
from .planner import LOOKAHEAD

MAX_STEER = math.pi/4


def pure_pursuit(current_x, current_y, heading, path, start_idx, speed=0.1):
    target = None
    idx = start_idx
    for i in range(start_idx, len(path)):
        pt = path[i]
        if math.hypot(pt[0]-current_x, pt[1]-current_y) > LOOKAHEAD:
            target = pt
            idx = i
            break
    if target is None:
        target = path[-1]
        idx = len(path) - 1
    desired_heading = math.atan2(target[1]-current_y, target[0]-current_x)
    steer = desired_heading - heading
    # normalize
    steer = (steer + math.pi) % (2*math.pi) - math.pi
    v = speed
    if abs(steer) > math.pi/6:
        v = 0.0
        steer = math.copysign(MAX_STEER, steer)
    return v, steer, idx