import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from landmark.msg import LandmarkObs
from duckie_bot_slam.ekf_slam.core_logic import EKFSLAM
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import csv
import math

def quaternion_to_yaw(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class EKFSLAMNode(Node):
    def __init__(self):
        super().__init__("ekf_slam")
        # self.declare_parameter('use_sim_time', True)
        self.marker_pub = self.create_publisher(MarkerArray, '/slam_landmarks', 10)
        self.path_pub = self.create_publisher(Path, '/ekf_path', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

        self.csv_file = open('simulation_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'odom_x', 'odom_y', 'ekf_x', 'ekf_y', 'v_cmd', 'w_cmd'])
        self.start_time = self.get_clock().now()
        

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        Q = np.diag([0.05, 0.05, 0.02]) 
        R = np.diag([0.1, 0.1]) 

        self.ekf = EKFSLAM(Q,R)
        self.ekf.initialize([0.0, 0.0, 0.0], np.eye(3))
        self.last_time = self.get_clock().now()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LandmarkObs, "/landmark_obs", self.landmark_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("EKF SLAM Node started. Path and Landmarks will show in RViz.")
    
    def odom_callback(self, msg):
        msg_time = msg.header.stamp
        
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.ekf.predict(v, w, dt)

        # Log data to CSV
        elapsed = (now - self.start_time).nanoseconds / 1e9
        self.csv_writer.writerow([elapsed, msg.pose.pose.position.x, msg.pose.pose.position.y, self.ekf.x[0], self.ekf.x[1], v, w])
    
        self.publish_pose()

        # 1. Get the current raw odometry heading (theta)
        current_odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # 2. Create the Transform message
        t = TransformStamped()
        t.header.stamp = msg_time
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # 1. Get current EKF heading and Odom heading
        ekf_theta = self.ekf.x[2]
        current_odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # 2. Calculate the difference (FLIP THE SIGN IF OPPOSITE)
        # Try changing the order of subtraction if they are mirrored
        diff_theta = ekf_theta - current_odom_yaw 

        # 3. Apply to the Transform message
        t.transform.rotation.z = np.sin(diff_theta / 2.0)
        t.transform.rotation.w = np.cos(diff_theta / 2.0)

        # 3. Calculate the translation (EKF Pose - Raw Odom Pose)
        # This "moves" the odom frame so the robot's base aligns with the EKF estimate
        t.transform.translation.x = self.ekf.x[0] - msg.pose.pose.position.x
        t.transform.translation.y = self.ekf.x[1] - msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # 4. Calculate the rotation correction
        diff_theta = self.ekf.x[2] - current_odom_yaw
        t.transform.rotation.z = np.sin(diff_theta / 2.0)
        t.transform.rotation.w = np.cos(diff_theta / 2.0)

        # 5. Send the dynamic transform
        self.tf_broadcaster.sendTransform(t)
    
    def landmark_callback(self, msg):
        self.ekf.update(msg.id, msg.range, msg.bearing)

    def publish_pose(self):
        now_msg = self.get_clock().now().to_msg()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = now_msg
        pose.pose.pose.position.x = float(self.ekf.x[0])
        pose.pose.pose.position.y = float(self.ekf.x[1])
        theta = self.ekf.x[2]
        pose.pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.pose.orientation.w = np.cos(theta / 2.0)

        full_cov = np.zeros(36)
        full_cov[0] = self.ekf.P[0, 0]; full_cov[7] = self.ekf.P[1, 1]; full_cov[35] = self.ekf.P[2, 2]
        pose.pose.covariance = full_cov.tolist()

        self.pose_pub.publish(pose)
        self.publish_markers()

        pose_stamped = PoseStamped()
        pose_stamped.header = pose.header
        pose_stamped.pose = pose.pose.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

    def calculate_map_errors(self):
        gt = {0: (0,0), 1: (10,0), 2: (4,3), 3: (8,5), 4: (2,7)}
        print("\n--- LANDMARK ESTIMATION ERRORS ---")
        for lid, idx in self.ekf.landmark_ids.items():
            if lid in gt:
                err = np.sqrt((self.ekf.x[idx]-gt[lid][0])**2 + (self.ekf.x[idx+1]-gt[lid][1])**2)
                print(f"Landmark {lid}: {err:.4f} meters")

    def publish_markers(self):
        marker_array = MarkerArray()
        for lid, idx in self.ekf.landmark_ids.items():
            marker = Marker()
            marker.header.frame_id = "map"; marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "landmarks"; marker.id = int(lid); marker.type = Marker.SPHERE
            marker.action = Marker.ADD; marker.pose.position.x = float(self.ekf.x[idx])
            marker.pose.position.y = float(self.ekf.x[idx+1]); marker.pose.position.z = 0.2
            marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
            marker.color.a = 1.0; marker.color.g = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

        mission_goals = [(2.0, 2.0), (8.0, 4.0), (5.0, 7.0)]
    
        for i, (gx, gy) in enumerate(mission_goals):
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "mission_goals"
            goal_marker.id = 100 + i # Offset ID to avoid conflict with landmarks
            goal_marker.type = Marker.CUBE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(gx)
            goal_marker.pose.position.y = float(gy)
            goal_marker.pose.position.z = 0.15
            goal_marker.scale.x = 0.4
            goal_marker.scale.y = 0.4
            goal_marker.scale.z = 0.4
            goal_marker.color.a = 0.8
            goal_marker.color.b = 1.0 # Blue
            marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.calculate_map_errors()
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()