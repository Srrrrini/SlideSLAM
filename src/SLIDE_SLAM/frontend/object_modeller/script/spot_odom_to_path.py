#!/usr/bin/env python3
"""
Converts Spot odometry to nav_msgs/Path in camera_init frame, aligned with FLIO trajectory.

Subscribes to: /spot_odom (Spot odom from bag), /Odometry (FLIO)
Publishes: /spot_path (nav_msgs/Path in camera_init frame)

On first synchronized pair, computes T_offset = T_flio * inv(T_spot) so Spot poses
are transformed into FLIO's camera_init frame. Publishes a continuous line path.
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def pose_to_matrix(pose):
    """Convert geometry_msgs Pose to 4x4 homogeneous matrix."""
    q = pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix to geometry_msgs Pose."""
    p = PoseStamped()
    p.pose.position.x = T[0, 3]
    p.pose.position.y = T[1, 3]
    p.pose.position.z = T[2, 3]
    R = T[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        p.pose.orientation.w = 0.25 / s
        p.pose.orientation.x = (R[2, 1] - R[1, 2]) * s
        p.pose.orientation.y = (R[0, 2] - R[2, 0]) * s
        p.pose.orientation.z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        p.pose.orientation.w = (R[2, 1] - R[1, 2]) / s
        p.pose.orientation.x = 0.25 * s
        p.pose.orientation.y = (R[0, 1] + R[1, 0]) / s
        p.pose.orientation.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        p.pose.orientation.w = (R[0, 2] - R[2, 0]) / s
        p.pose.orientation.x = (R[0, 1] + R[1, 0]) / s
        p.pose.orientation.y = 0.25 * s
        p.pose.orientation.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        p.pose.orientation.w = (R[1, 0] - R[0, 1]) / s
        p.pose.orientation.x = (R[0, 2] + R[2, 0]) / s
        p.pose.orientation.y = (R[1, 2] + R[2, 1]) / s
        p.pose.orientation.z = 0.25 * s
    return p


class SpotOdomToPath:
    def __init__(self):
        rospy.init_node('spot_odom_to_path', anonymous=False)
        self.first_flio_pose = None  # Store first FLIO pose for alignment
        self.first_spot_pose = None  # Store first Spot pose for alignment
        self.T_offset = None
        self.path_poses = []
        self.spot_buffer = []  # Buffer Spot odom msgs until we have alignment

        self.path_pub = rospy.Publisher('/spot_path', Path, queue_size=1, latch=False)

        rospy.Subscriber('/Odometry', Odometry, self.flio_cb, queue_size=10)
        rospy.Subscriber('/spot_odom', Odometry, self.spot_cb, queue_size=10)

        rospy.loginfo("spot_odom_to_path: Subscribed to /Odometry, /spot_odom; publishing /spot_path")

    def _try_align(self):
        """Compute offset from first FLIO and first Spot poses (align origins)."""
        if self.T_offset is None and self.first_flio_pose is not None and self.first_spot_pose is not None:
            T_flio = pose_to_matrix(self.first_flio_pose)
            T_spot = pose_to_matrix(self.first_spot_pose)
            self.T_offset = T_flio @ np.linalg.inv(T_spot)
            rospy.loginfo("spot_odom_to_path: Aligned Spot trajectory with FLIO frame (first poses)")

    def flio_cb(self, msg):
        if self.first_flio_pose is None:
            self.first_flio_pose = msg.pose.pose
        self._try_align()
        if self.T_offset is not None and self.spot_buffer:
            # Transform and publish buffered Spot poses
            for m in self.spot_buffer:
                T_s = pose_to_matrix(m.pose.pose)
                T_cam = self.T_offset @ T_s
                ps = matrix_to_pose(T_cam)
                ps.header.frame_id = "camera_init"
                ps.header.stamp = m.header.stamp
                self.path_poses.append(ps)
            path_msg = Path()
            path_msg.header.frame_id = "camera_init"
            path_msg.header.stamp = self.spot_buffer[-1].header.stamp
            path_msg.poses = self.path_poses
            self.path_pub.publish(path_msg)
            self.spot_buffer = []

    def spot_cb(self, msg):
        if self.first_spot_pose is None:
            self.first_spot_pose = msg.pose.pose
        self._try_align()

        if self.T_offset is None:
            self.spot_buffer.append(msg)
            return

        T_spot = pose_to_matrix(msg.pose.pose)
        T_camera_init = self.T_offset @ T_spot
        ps = matrix_to_pose(T_camera_init)
        ps.header.frame_id = "camera_init"
        ps.header.stamp = msg.header.stamp

        self.path_poses.append(ps)

        path_msg = Path()
        path_msg.header.frame_id = "camera_init"
        path_msg.header.stamp = msg.header.stamp
        path_msg.poses = self.path_poses
        self.path_pub.publish(path_msg)


def main():
    try:
        node = SpotOdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
