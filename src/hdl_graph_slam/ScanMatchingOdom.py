#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import rospy
import tf  # Add this import line
from geometry_msgs.msg import TransformStamped


class ScanMatchingOdom:
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            "/scan_matching_odometry/transform", TransformStamped, self.callback
        )

        self.pose_file_path = (
            "/home/malik/catkin_ws_scan/src/hdl_graph_slam/output/pose.txt"
        )
        self.pose_file = open(self.pose_file_path, "w")
        self.pose_file.write("#timestamp tx ty tz qx qy qz qw\n")

    def callback(self, odom_msg):
        try:
            self.odom_msg = odom_msg
            self.save_pose_data()
        except Exception as e:
            rospy.logerr(f"Error during pose saving: {e}")

    def save_pose_data(self):
        timestamp = self.odom_msg.header.stamp.to_sec()
        tx = self.odom_msg.transform.translation.x
        ty = self.odom_msg.transform.translation.y
        tz = self.odom_msg.transform.translation.z
        qx = self.odom_msg.transform.rotation.x
        qy = self.odom_msg.transform.rotation.y
        qz = self.odom_msg.transform.rotation.z
        qw = self.odom_msg.transform.rotation.w
        self.pose_file.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
        self.pose_file.flush()

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node("ScanMatchingOdom")
    node = ScanMatchingOdom()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
