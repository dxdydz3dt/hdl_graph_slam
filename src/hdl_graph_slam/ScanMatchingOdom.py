#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import rospy
import tf  # Add this import line
from geometry_msgs.msg import TransformStamped
import time  # Import the time module


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
        self.start_time = None  # Initialize the start time

    def callback(self, odom_msg):
        try:
            if self.start_time is None:
                self.start_time = time.time()
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

    def finish(self):
        if self.start_time is not None:
            end_time = time.time()
            total_processing_time = end_time - self.start_time
            with open("processing_time.log", "a") as log_file:
                log_file.write(
                    f"Total processing time: {total_processing_time:.2f} seconds\n"
                )
            self.pose_file.close()


def main():
    rospy.init_node("ScanMatchingOdom")
    node = ScanMatchingOdom()
    rate = rospy.Rate(10.0)

    try:
        while not rospy.is_shutdown():
            node.spin()
            rate.sleep()
    finally:
        node.finish()


if __name__ == "__main__":
    main()
