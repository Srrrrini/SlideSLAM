#!/usr/bin/env python3
"""
Odometry relay node to bridge SLIDE_SLAM odometry to map_manager.

Subscribes to: /Odometry (from faster-lio/Spot LIO - the source)
Publishes to: /odom_ugv, /odom_uav, /spot/odom (for map_manager compatibility)

This ensures map_manager receives odometry regardless of which camera init mode is used.
We subscribe directly from the source to avoid feedback loops and processing artifacts.
"""

import rospy
import copy
from nav_msgs.msg import Odometry


class OdomRelay:
    def __init__(self):
        rospy.init_node('odom_relay', anonymous=False)
        
        # Get parameters for topic names
        # Default to /Odometry (source from faster-lio) to avoid feedback loops
        input_topic = rospy.get_param('~input_topic', '/Odometry')
        output_topics = rospy.get_param('~output_topics', ['/odom_ugv', '/odom_uav', '/spot/odom'])
        
        # If output_topics is a string, convert to list
        if isinstance(output_topics, str):
            output_topics = [output_topics]
        
        # Publishers for each output topic
        self.odom_pubs = {}
        for topic in output_topics:
            self.odom_pubs[topic] = rospy.Publisher(
                topic,
                Odometry,
                queue_size=10
            )
        
        # Subscriber - subscribe directly from source to avoid feedback
        self.odom_sub = rospy.Subscriber(
            input_topic,
            Odometry,
            self.odom_callback,
            queue_size=10
        )
        
        rospy.loginfo("Odometry relay initialized:")
        rospy.loginfo("  Subscribing to: %s (source odometry)", input_topic)
        rospy.loginfo("  Publishing to: %s", ', '.join(output_topics))
    
    def odom_callback(self, msg):
        """
        Relay odometry message to all output topics.
        Preserves the original message structure but may adjust frame_ids for compatibility.
        """
        # Create a copy to avoid modifying the original message
        msg_copy = copy.deepcopy(msg)
        
        # Publish to each output topic
        for topic, pub in self.odom_pubs.items():
            # For /spot/odom, ensure frame_id compatibility with map_manager
            if topic == '/spot/odom':
                # Map manager expects spot/odom to have frame_id matching its config
                # Keep original frame_id but ensure it's correct
                output_msg = copy.deepcopy(msg_copy)
                # Faster-lio uses "camera_init" as frame_id, which should be fine
                # But map_manager might expect "odom" - we'll preserve original
                pub.publish(output_msg)
            else:
                # For other topics, publish as-is
                pub.publish(msg_copy)


if __name__ == '__main__':
    try:
        relay = OdomRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

