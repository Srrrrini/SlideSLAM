#!/usr/bin/env python3
"""
Bridge node to convert SLIDE_SLAM object poses to map_manager format.

Subscribes to: /robot0/chair_cuboids (visualization_msgs/MarkerArray)
Publishes to: /semantic_objects (map_manager/semanticObjArrayMsg)

This node converts RViz markers from SLIDE_SLAM to the semantic object format
expected by map_manager, without modifying the SLIDE_SLAM pipeline.
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from map_manager.msg import semanticObjArrayMsg, semanticObjMsg
from geometry_msgs.msg import Point


class SlideSlamToMapManagerBridge:
    def __init__(self):
        rospy.init_node('slide_slam_to_map_manager_bridge', anonymous=False)
        
        # Class name to label_id mapping (from process_cloud_node_indoor_cls_info.yaml)
        self.class_to_label_id = {
            'chair': 1,
            'table': 2,
            'tv': 3,
            # Add more classes as needed
        }
        
        # Publisher for map_manager
        self.semantic_obj_pub = rospy.Publisher(
            '/semantic_objects',
            semanticObjArrayMsg,
            queue_size=10
        )
        
        # Subscriber to SLIDE_SLAM object poses
        # Use world frame markers (chair_cuboids) not body frame (chair_cuboids_body)
        self.marker_sub = rospy.Subscriber(
            '/robot0/chair_cuboids',
            MarkerArray,
            self.marker_callback,
            queue_size=10
        )
        
        rospy.loginfo("Bridge node initialized:")
        rospy.loginfo("  Subscribing to: /robot0/chair_cuboids")
        rospy.loginfo("  Publishing to: /semantic_objects")
        rospy.loginfo("  Class mappings: %s", self.class_to_label_id)
    
    def marker_callback(self, marker_array_msg):
        """
        Convert MarkerArray to semanticObjArrayMsg.
        
        Args:
            marker_array_msg: visualization_msgs/MarkerArray from SLIDE_SLAM
        """
        semantic_obj_array = semanticObjArrayMsg()
        semantic_obj_array.semanticObjs = []
        
        for marker in marker_array_msg.markers:
            # Skip if marker is not a valid object (e.g., DELETE action)
            if marker.action == Marker.DELETE:
                continue
            
            # Get class name from marker namespace
            class_name = marker.ns.lower()  # e.g., "chair", "table"
            
            # Map class name to label_id
            if class_name not in self.class_to_label_id:
                rospy.logwarn_throttle(
                    5, 
                    "Unknown class name '%s', skipping. Known classes: %s", 
                    class_name, 
                    list(self.class_to_label_id.keys())
                )
                continue
            
            label_id = self.class_to_label_id[class_name]
            
            # Create semantic object message
            semantic_obj = semanticObjMsg()
            semantic_obj.label_id = label_id
            
            # Extract position from marker pose
            semantic_obj.position = [
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ]
            
            # Extract size from marker scale
            # Note: marker.scale is Vector3 (x, y, z) representing dimensions
            semantic_obj.size = [
                marker.scale.x,  # length
                marker.scale.y,  # width
                marker.scale.z   # height
            ]
            
            # Set optional fields
            semantic_obj.height_covered = False  # Default value
            semantic_obj.view_angles = []  # Empty array (not used by map_manager for object poses)
            semantic_obj.voxels = []  # Empty array (not used for object poses)
            
            semantic_obj_array.semanticObjs.append(semantic_obj)
        
        # Publish the converted message
        if len(semantic_obj_array.semanticObjs) > 0:
            self.semantic_obj_pub.publish(semantic_obj_array)
            rospy.loginfo_throttle(
                2,
                "Published %d semantic objects to /semantic_objects",
                len(semantic_obj_array.semanticObjs)
            )
        else:
            rospy.logdebug("No valid semantic objects to publish")


if __name__ == '__main__':
    try:
        bridge = SlideSlamToMapManagerBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

