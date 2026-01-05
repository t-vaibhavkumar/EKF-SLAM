import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped




def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # -------- Goals --------
        self.goals = [
            (2.0, 2.0),
            (8.0, 4.0),
            (5.0, 7.0)
        ]
        self.current_goal_idx = 0

        # -------- Control gains --------
        self.k_rho = 0.6      # linear velocity gain
        self.k_alpha = 1.5    # angular velocity gain

        self.goal_tolerance = 0.3  # meters

        # -------- Robot state --------
        self.x = None
        self.y = None
        self.theta = None

        # -------- ROS interfaces --------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_pose',
            self.pose_callback,
            10
        )

        # Run controller at fixed rate
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Waypoint controller started.")

    # --------------------------------------------------
    # Pose callback
    # --------------------------------------------------
    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------
    def control_loop(self):
        if self.x is None:
            return  # EKF pose not available yet

        if self.current_goal_idx >= len(self.goals):
            self.stop_robot()
            self.get_logger().info("All goals reached.")
            return

        goal_x, goal_y = self.goals[self.current_goal_idx]

        dx = goal_x - self.x
        dy = goal_y - self.y

        rho = np.sqrt(dx*dx + dy*dy)
        alpha = normalize_angle(np.arctan2(dy, dx) - self.theta)

        # -------- Check goal reached --------
        if rho < self.goal_tolerance:
            self.get_logger().info(
                f"Reached goal {self.current_goal_idx + 1}: "
                f"({goal_x}, {goal_y})"
            )
            self.current_goal_idx += 1
            self.stop_robot()
            return

        # -------- Control law --------
        v = self.k_rho * rho
        w = self.k_alpha * alpha

        # Limit velocities
        v = np.clip(v, 0.0, 0.5)
        w = np.clip(w, -1.5, 1.5)

        # -------- Publish command --------
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()