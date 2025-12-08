#!/usr/bin/env python

"""
Detection Adapter Node for Map Manager Integration

Subscribes to SLIDE_SLAM's segmented point cloud and converts it to
vision_msgs/Detection2DArray for map_manager's raw detection pipeline.

Processes per-pixel semantic labels and bounding boxes from the point cloud
and publishes standard ROS detection messages.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D, Point
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2


class DetectionAdapter:
    def __init__(self):
        rospy.init_node('detection_adapter', anonymous=False)
        
        # Parameters
        self.input_pointcloud_topic = rospy.get_param('~input_pointcloud_topic', '/sem_detection/pointcloud')
        self.output_detection_topic = rospy.get_param('~output_detection_topic', '/yolo/detection')
        self.min_points_threshold = rospy.get_param('~min_points_threshold', 50)  # Minimum points to consider as valid detection
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.3)
        
        # Image dimensions (for bounding box normalization if needed)
        self.image_width = rospy.get_param('~image_width', 1280)
        self.image_height = rospy.get_param('~image_height', 720)
        
        # Subscriber (queue_size=1 to process only latest, drop old messages)
        self.pc_sub = rospy.Subscriber(
            self.input_pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1,
            buff_size=2**24  # 16MB buffer to prevent message loss
        )
        
        # Publishers
        self.detection_pub = rospy.Publisher(
            self.output_detection_topic,
            Detection2DArray,
            queue_size=1  # Latest only
        )
        
        # Processing stats
        self.last_process_time = rospy.Time.now()
        self.process_count = 0
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        rospy.loginfo(f"Detection adapter initialized:")
        rospy.loginfo(f"  Input:  {self.input_pointcloud_topic}")
        rospy.loginfo(f"  Output: {self.output_detection_topic}")
        rospy.loginfo(f"  Min points threshold: {self.min_points_threshold}")
        rospy.loginfo(f"  Confidence threshold: {self.confidence_threshold}")
        
    def pointcloud_callback(self, pc_msg):
        """
        Process incoming point cloud with semantic labels and extract 2D bounding boxes.
        
        Expected fields: x, y, z, intensity (label_id), id (instance_id), confidence
        
        OPTIMIZED: Skip processing if callback rate is too high (< 100ms between calls)
        """
        try:
            # Throttle processing to max 10Hz to avoid CPU overload
            current_time = rospy.Time.now()
            time_since_last = (current_time - self.last_process_time).to_sec()
            if time_since_last < 0.1:  # Less than 100ms since last processing
                return  # Skip this frame
            self.last_process_time = current_time
            self.process_count += 1
            # Read point cloud fields
            pc_data = []
            for point in pc2.read_points(pc_msg, field_names=("x", "y", "z", "intensity", "id", "confidence"), skip_nans=True):
                pc_data.append(point)
            
            if len(pc_data) == 0:
                rospy.logwarn_throttle(5, "Received empty point cloud")
                return
            
            pc_array = np.array(pc_data)
            
            # Extract fields
            # intensity field contains the label_id (class)
            # id field contains the instance_id
            # confidence field contains detection confidence
            labels = pc_array[:, 3]  # intensity = label_id
            instance_ids = pc_array[:, 4]  # id = instance_id
            confidences = pc_array[:, 5]  # confidence
            
            # Reconstruct pixel indices from point cloud dimensions
            # The point cloud is organized (has width and height)
            if pc_msg.height > 1:
                # Organized point cloud
                total_points = pc_msg.height * pc_msg.width
                pixel_indices = np.arange(total_points).reshape(pc_msg.height, pc_msg.width)
            else:
                rospy.logwarn_throttle(5, "Point cloud is not organized, cannot extract 2D bounding boxes")
                return
            
            # Group points by instance_id
            unique_instances = np.unique(instance_ids)
            unique_instances = unique_instances[unique_instances > 0]  # Filter out background (id=0)
            
            detections_msg = Detection2DArray()
            detections_msg.header = pc_msg.header
            
            for inst_id in unique_instances:
                # Get mask for this instance
                inst_mask = (instance_ids == inst_id)
                
                # Skip if too few points
                if np.sum(inst_mask) < self.min_points_threshold:
                    continue
                
                # Get label and average confidence for this instance
                inst_labels = labels[inst_mask]
                inst_confidences = confidences[inst_mask]
                
                # Use the most common label
                unique_labels, label_counts = np.unique(inst_labels, return_counts=True)
                label_id = int(unique_labels[np.argmax(label_counts)])
                
                # Skip background
                if label_id == 0:
                    continue
                
                # Average confidence
                avg_confidence = float(np.mean(inst_confidences))
                
                # Skip low confidence detections
                if avg_confidence < self.confidence_threshold:
                    continue
                
                # Find 2D bounding box in pixel space
                # Reshape mask back to image dimensions
                inst_mask_2d = inst_mask.reshape(pc_msg.height, pc_msg.width)
                
                # Get pixel coordinates where mask is True
                v_coords, u_coords = np.where(inst_mask_2d)
                
                if len(u_coords) == 0 or len(v_coords) == 0:
                    continue
                
                # Calculate bounding box
                u_min, u_max = np.min(u_coords), np.max(u_coords)
                v_min, v_max = np.min(v_coords), np.max(v_coords)
                
                # Bounding box center and size
                center_x = (u_min + u_max) / 2.0
                center_y = (v_min + v_max) / 2.0
                size_x = u_max - u_min
                size_y = v_max - v_min
                
                # Create Detection2D message
                detection = Detection2D()
                detection.header = pc_msg.header
                
                # Bounding box
                detection.bbox = BoundingBox2D()
                detection.bbox.center = Pose2D()
                detection.bbox.center.x = center_x
                detection.bbox.center.y = center_y
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = size_x
                detection.bbox.size_y = size_y
                
                # Object hypothesis (class and confidence)
                # Note: ROS 1 Noetic vision_msgs has id/score directly in ObjectHypothesisWithPose
                hypothesis_with_pose = ObjectHypothesisWithPose()
                hypothesis_with_pose.id = int(label_id)  # Class ID as int64
                hypothesis_with_pose.score = avg_confidence
                detection.results.append(hypothesis_with_pose)
                
                # Create binary mask image for this instance
                # Map_manager requires this mask to filter depth pixels
                mask_image = np.zeros((pc_msg.height, pc_msg.width), dtype=np.uint8)
                mask_image[inst_mask_2d] = 255  # Set instance pixels to 255 (white)
                
                # Debug: Count white pixels
                white_pixel_count = np.sum(mask_image == 255)
                rospy.loginfo_throttle(3, f"Detection {int(inst_id)}: bbox=({center_x:.1f},{center_y:.1f}), size=({size_x:.1f}x{size_y:.1f}), mask_pixels={white_pixel_count}")
                
                # Convert mask to ROS Image message
                try:
                    detection.source_img = self.bridge.cv2_to_imgmsg(mask_image, encoding="mono8")
                    detection.source_img.header = pc_msg.header
                except Exception as e:
                    rospy.logwarn(f"Failed to convert mask to image: {e}")
                    continue
                
                # Add to detections array
                detections_msg.detections.append(detection)
            
            # Publish detections
            if len(detections_msg.detections) > 0:
                self.detection_pub.publish(detections_msg)
                rospy.loginfo_throttle(3, f"Published {len(detections_msg.detections)} detections")
            else:
                rospy.logdebug_throttle(3, "No valid detections to publish")
                
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def run(self):
        rospy.loginfo("Detection adapter node running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DetectionAdapter()
        node.run()
    except rospy.ROSInterruptException:
        pass

