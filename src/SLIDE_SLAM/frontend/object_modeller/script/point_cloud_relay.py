#!/usr/bin/env python3
"""
Simple relay node to remap point cloud topics for map_manager.

This node relays point clouds from SLIDE_SLAM topics to map_manager expected topics.
Can be used if direct topic remapping in launch files is not sufficient.
"""

import rospy
from sensor_msgs.msg import PointCloud2


class PointCloudRelay:
    def __init__(self):
        rospy.init_node('point_cloud_relay', anonymous=False)
        
        # Get parameters for topic names
        input_topic = rospy.get_param('~input_topic', '/cloud_registered')
        output_topic = rospy.get_param('~output_topic', '/point_cloud_ugv')
        
        # Publisher
        self.pc_pub = rospy.Publisher(
            output_topic,
            PointCloud2,
            queue_size=10
        )
        
        # Subscriber
        self.pc_sub = rospy.Subscriber(
            input_topic,
            PointCloud2,
            self.pc_callback,
            queue_size=10
        )
        
        rospy.loginfo("Point cloud relay initialized:")
        rospy.loginfo("  Subscribing to: %s", input_topic)
        rospy.loginfo("  Publishing to: %s", output_topic)
    
    def pc_callback(self, msg):
        """Relay point cloud message."""
        self.pc_pub.publish(msg)


if __name__ == '__main__':
    try:
        relay = PointCloudRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

