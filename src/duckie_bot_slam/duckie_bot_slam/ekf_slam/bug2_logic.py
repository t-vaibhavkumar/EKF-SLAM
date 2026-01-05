import math
import numpy as np

class TangentialBug2:
    def __init__(self, goals):
        self.goals = goals
        self.goal_idx = 0
        self.state = 'ALIGN_TO_GOAL'
        self.m_line_start = None
        self.hit_point = None
        self.hit_dist_to_goal = float('inf')
        
        # Parameters
        self.dist_tol = 0.3
        self.angle_tol = 0.1
        self.m_line_tol = 0.2
        self.safe_dist = 0.6
        self.wall_follow_dist = 0.5

    def get_current_goal(self):
        return self.goals[self.goal_idx]

    def dist_to_line(self, p1, p2, p3):
        # Calculates distance of point p3 from line segment p1-p2
        x1, y1 = p1; x2, y2 = p2; x3, y3 = p3
        num = abs((y2 - y1) * x3 - (x2 - x1) * y3 + x2 * y1 - y2 * x1)
        den = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        return num / den if den != 0 else 0

    def step(self, curr_pose, laser_data):
        rx, ry, ryaw = curr_pose
        gx, gy = self.get_current_goal()
        
        dx, dy = gx - rx, gy - ry
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(desired_yaw - ryaw), math.cos(desired_yaw - ryaw))

        # Check Goal Reach
        if dist_to_goal < self.dist_tol:
            self.goal_idx = (self.goal_idx + 1) % len(self.goals)
            self.state = 'ALIGN_TO_GOAL'
            self.m_line_start = None
            return 0.0, 0.0, True # Goal reached flag

        # Sensor readings
        front = laser_data['front']
        right = laser_data['right']

        v, w = 0.0, 0.0

        if self.state == 'ALIGN_TO_GOAL':
            if abs(yaw_err) > self.angle_tol:
                w = np.clip(1.5 * yaw_err, -0.6, 0.6)
            else:
                self.state = 'GO_TO_GOAL'
                if self.m_line_start is None:
                    self.m_line_start = (rx, ry)

        elif self.state == 'GO_TO_GOAL':
            if front < self.safe_dist:
                self.state = 'WALL_FOLLOW'
                self.hit_point = (rx, ry)
                self.hit_dist_to_goal = dist_to_goal
            else:
                v = 0.2
                w = 0.5 * yaw_err

        elif self.state == 'WALL_FOLLOW':
            d_line = self.dist_to_line(self.m_line_start, (gx, gy), (rx, ry))
            d_from_hit = math.sqrt((rx - self.hit_point[0])**2 + (ry - self.hit_point[1])**2)

            # --- MODIFIED LEAVE CONDITION ---
            # 1. Increased tolerance to 0.3m to account for EKF noise
            # 2. Relaxed distance check by 0.1m to ensure exit triggers
            if d_line < 0.3 and dist_to_goal < (self.hit_dist_to_goal - 0.1) and d_from_hit > 0.5:
                self.state = 'ALIGN_TO_GOAL'
            else:
                if front < self.safe_dist:
                    w = 0.6 # Turn away from wall
                else:
                    v = 0.15
                    # Maintain distance from wall on the right
                    w = -1.5 * (right - self.wall_follow_dist)

        return v, w, False