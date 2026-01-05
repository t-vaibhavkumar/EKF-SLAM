import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from landmark.msg import LandmarkObs


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LandmarkSensor(Node):
    """
    Simulated landmark sensor.
    Subscribes to /odom and publishes noisy range-bearing
    measurements to known landmarks on /landmark_obs.
    """

    def __init__(self):
        super().__init__('landmark_sensor')

        #Ground-truth landmark position 
        self.landmarks = {
            0: (0.0, 0.0),
            1: (10.0, 0.0),
            2: (4.0, 3.0),
            3: (8.0, 5.0),
            4: (2.0, 7.0)
        }

        #Sensor noise
        self.sigma_r = 0.10      # meters
        self.sigma_phi = 0.05    # radians


        self.landmark_pub = self.create_publisher(
            LandmarkObs,
            '/landmark_obs',
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("Landmark sensor node started (using /odom).")


    def odom_callback(self, msg: Odometry):
        """
        Called every time odometry is published.
        Uses robot pose to compute landmark observations.
        """

        # --- Extract robot pose ---
        pose = msg.pose.pose
        x_r = pose.position.x
        y_r = pose.position.y

        # Quaternion → yaw
        q = pose.orientation
        theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # --- Generate observations for each landmark ---
        for landmark_id, (x_l, y_l) in self.landmarks.items():

            dx = x_l - x_r
            dy = y_l - y_r

            # True range & bearing
            r = np.sqrt(dx * dx + dy * dy)
            phi = np.arctan2(dy, dx) - theta

            # Add sensor noise
            r_noisy = r + np.random.normal(0.0, self.sigma_r)
            phi_noisy = phi + np.random.normal(0.0, self.sigma_phi)
            phi_noisy = normalize_angle(phi_noisy)

            # Publish observation
            obs = LandmarkObs()
            obs.id = landmark_id
            obs.range = float(r_noisy)
            obs.bearing = float(phi_noisy)

            self.landmark_pub.publish(obs)



def main(args=None):
    rclpy.init(args=args)
    node = LandmarkSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
