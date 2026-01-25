#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import numpy as np
import torch
import cv2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from sloam_msgs.msg import syncPcOdom
import yaml

bridge = CvBridge()


# observation 1: in the bag file, color and depth images can go out of sync
# observation 2: might be able to use aligned depth?

class sem_detection:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('sem_detection')

        self.prev_time = rospy.Time.now()
        rospack = rospkg.RosPack()
        
        # Get YOLO model from parameter (configurable via launch file)
        yolo_model = rospy.get_param('~yolo_model', 'yolov8x-seg.pt')
        self.model_path = rospack.get_path('object_modeller') + '/models/' + yolo_model
        rospy.loginfo(f"[sem_detection] Using YOLO model: {yolo_model}")
        
        # Original model (commented out - now configured via launch parameter):
        # self.model_path = rospack.get_path('object_modeller') + '/models/yolov8x-seg.pt'
        
        ##################################### PARAMS / CONFIG #####################################
        # GPU/CPU toggle
        self.use_gpu = rospy.get_param('~use_gpu', False)
        
        # CPU optimization: use more threads when running on CPU
        if not self.use_gpu:
            num_threads = rospy.get_param('~cpu_threads', 8)  # Use 8 threads by default
            torch.set_num_threads(num_threads)
            cv2.setNumThreads(num_threads)
            rospy.loginfo(f"[sem_detection] Set CPU threads to {num_threads} for PyTorch and OpenCV")
        
        # Set device based on use_gpu parameter
        if self.use_gpu:
            # Check if CUDA is actually available
            if torch.cuda.is_available():
                self.device = '0'  # First GPU
                rospy.loginfo("[sem_detection] Using GPU for YOLO inference")
            else:
                self.device = 'cpu'
                rospy.logwarn("[sem_detection] GPU requested but CUDA unavailable - using CPU")
        else:
            self.device = 'cpu'
            rospy.loginfo("[sem_detection] Using CPU for YOLO inference")
        
        self.yolo = YOLO(self.model_path)
        
        self.sim = False
        # RealSense D435i camera intrinsics
        self.color_fx = rospy.get_param("~fx", 603.7166748046875)
        self.color_fy = rospy.get_param("~fy", 603.9064331054688)
        self.color_cx = rospy.get_param("~cx", 314.62518310546875)
        self.color_cy = rospy.get_param("~cy", 244.9166717529297) 
        self.k_depth_scaling_factor = rospy.get_param("~k_depth_scaling_factor", 1000.0)
        
        if self.sim:
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            self.depth_scale = 1
        else:
            self.depth_scale = 1 / self.k_depth_scaling_factor

        self.desired_rate = rospy.get_param('~desired_rate', 2.0)

        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.4)
        ##################################### PARAMS / CONFIG #####################################

        # -----------------------------------------------------------------
        # Define the parameters for the cylindrical camera model.
        # Use the actual values from your sensor calibration.
        self.W_img = 1280.0  # Total width of the panorama image
        self.f_c = 219.4734627    
        self.y_c_0 = 375.0  
        # -----------------------------------------------------------------



        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.color_fx)
        print("fy: ", self.color_fy)
        print("cx: ", self.color_cx)
        print("cy: ", self.color_cy)

        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw/')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw/')
        self.aligned_depth_topic = rospy.get_param('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw')
        # self.odom_topic = rospy.get_param("~odom_topic", "/dragonfly67/quadrotor_ukf/control_odom")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_ugv")
        self.sync_odom_measurements = rospy.get_param('~sync_odom_measurements', True)
        self.sync_pc_odom_topic = rospy.get_param('~sync_pc_odom_topic', '/sem_detection/sync_pc_odom')
        self.pc_topic = rospy.get_param('~pc_topic', '/sem_detection/pointcloud')
        self.detection_image_topic = rospy.get_param('~detection_image_topic', '/sem_detection/detections_image')

        # Subscriber and publisher
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)

        # Subscribe to the depth image
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.aligned_depth_sub = message_filters.Subscriber(self.aligned_depth_topic, Image)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        self.pc_pub_ = rospy.Publisher(self.pc_topic, PointCloud2, queue_size=1)
        self.synced_pc_pub_ = rospy.Publisher(self.sync_pc_odom_topic, syncPcOdom, queue_size=1)
        self.detection_image_pub = rospy.Publisher(self.detection_image_topic, Image, queue_size=1)
        
        # Publisher for depth image with YOLO bounding boxes overlay (for debugging)
        self.depth_with_detections_pub = rospy.Publisher('/robot0/sem_detection/depth_with_detections', Image, queue_size=1)


        # Synchronize the two image topics with a time delay of 0.1 seconds
        if (self.sync_odom_measurements):
            rospy.loginfo("Syncing rgb, aligned depth and odom")
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.rgb_aligned_depth_odom_callback)
        else:
            ts = message_filters.TimeSynchronizer([self.rgb_sub, self.aligned_depth_sub], 5)
            ts.registerCallback(self.rgb_aligned_depth_callback)

        self.pc_fields_ = self.make_fields()

        self.cls_config_path = rospack.get_path('scan2shape_launch') + '/config/process_cloud_node_indoor_cls_info.yaml'

        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)
               
        # convert the "table" to "dining table" for yolov8
        self.cls = {}
        for key, value in self.cls_data_all.items():
            if key == "table":
                self.cls["dining table"] = value["id"]
            else:
                self.cls[key] = value["id"]
        
        print(f"[DEBUG] Configured class mapping: {self.cls}")        


    def rgb_aligned_depth_odom_callback(self, rgb, aligned_depth, odom):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            rospy.loginfo_throttle(3, f"Time elapsed since last depth rgb callback is: {(rospy.Time.now() - self.prev_time).to_sec()}")
            rospy.loginfo_throttle(3, f"Skipping current depth image to get desired rate of {self.desired_rate}")
            return
        else:
            self.prev_time = rospy.Time.now()

        try:
            # Convert ROS image message to OpenCV image
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        # TODO(ankit): Edited this
        detections = self.yolo.predict(color_img, show=False, device=self.device, verbose=False)

        # 2. open img_size * 2 array save class and id
        # pc_pos = np.zeros([color_img.shape[0], color_img.shape[1], 3])
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])
        
        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections is of size 1
            # detections[0] contains everything
            # masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                # print(detection.masks.masks)     # masks, (N, H, W)
                # print(detection.masks.segments)  # bounding coordinates of masks, List[segment] * N
                # print(detection.boxes.conf)   # confidence score, (N, 1)
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    # TODO(ankit): Edited here
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()
                    
                    # Resize mask to match image resolution if needed
                    if cur_mask.shape != (color_img.shape[0], color_img.shape[1]):
                        cur_mask = cv2.resize(cur_mask.astype(np.uint8), 
                                              (color_img.shape[1], color_img.shape[0]),  # (width, height)
                                              interpolation=cv2.INTER_NEAREST).astype(cur_mask.dtype)
                        print(f"[DEBUG] Resized mask from {detection.masks.masks[i].shape} to {cur_mask.shape}")

                    mask_pos = np.where(cur_mask != 0)
                    assigned_label = self.get_cls_label(cls_str)
                    label[mask_pos] = assigned_label
                    print(f"[YOLO] detected: '{cls_str}' -> label={assigned_label} (0=background/ignored)")
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])
                    
                    # DEBUG: Check mask location vs depth
                    if assigned_label > 0 and len(mask_pos[0]) > 0:
                        mask_rows = mask_pos[0]  # row indices (y)
                        mask_cols = mask_pos[1]  # col indices (x)
                        mask_depth_vals = depth_img[mask_pos] * self.depth_scale
                        
                        # Get bounding box
                        box = detection.boxes.xyxy[i].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = box
                        bbox_depth = depth_img[y1:y2, x1:x2] * self.depth_scale
                        bbox_valid = bbox_depth[(bbox_depth > 0.1) & (bbox_depth < 50)]
                        
                        print(f"[DEBUG MASK] Mask shape: {cur_mask.shape}, Depth shape: {depth_img.shape}")
                        print(f"[DEBUG MASK] Mask pixel rows: min={np.min(mask_rows)}, max={np.max(mask_rows)} (image height={depth_img.shape[0]})")
                        print(f"[DEBUG MASK] Mask pixel cols: min={np.min(mask_cols)}, max={np.max(mask_cols)} (image width={depth_img.shape[1]})")
                        print(f"[DEBUG MASK] Mask depth values: min={np.min(mask_depth_vals):.2f}, max={np.max(mask_depth_vals):.2f}")
                        print(f"[DEBUG MASK] BBox [{x1},{y1}]-[{x2},{y2}]: {len(bbox_valid)} valid depth pixels out of {bbox_depth.size}")
        
        # Publish annotated image with bounding boxes overlay
        # The plot() method works even when there are no detections (returns original image)
        try:
            annotated_img = detections[0].plot()
            annotated_img_msg = bridge.cv2_to_imgmsg(annotated_img, "bgr8")
            annotated_img_msg.header = rgb.header
            self.detection_image_pub.publish(annotated_img_msg)
        except Exception as e:
            rospy.logerr("Error publishing detection image: %s", e)
        
        # Publish depth image with YOLO bounding boxes overlay for debugging
        try:
            # Normalize depth for visualization (0-255)
            depth_scaled = depth_img * self.depth_scale
            
            # Create color visualization: valid depth = grayscale, invalid = RED
            depth_viz_color = np.zeros((depth_img.shape[0], depth_img.shape[1], 3), dtype=np.uint8)
            
            # Mark invalid depth (>50m) as RED
            invalid_mask = depth_scaled > 50.0
            depth_viz_color[invalid_mask] = [0, 0, 255]  # Red for invalid
            
            # Valid depth: normalize 0-20m to grayscale
            valid_mask = ~invalid_mask
            valid_depth = np.clip(depth_scaled, 0, 20)
            valid_depth_normalized = (valid_depth / 20.0 * 255).astype(np.uint8)
            depth_viz_color[valid_mask, 0] = valid_depth_normalized[valid_mask]  # B
            depth_viz_color[valid_mask, 1] = valid_depth_normalized[valid_mask]  # G
            depth_viz_color[valid_mask, 2] = valid_depth_normalized[valid_mask]  # R
            
            # Draw bounding boxes from YOLO detections
            if len(detections[0]) != 0:
                for detection in detections:
                    for i in range(len(detection.boxes)):
                        cls_int = int(detection.boxes.cls[i])
                        cls_str = self.yolo.names[cls_int]
                        assigned_label = self.get_cls_label(cls_str)
                        conf_val = float(detection.boxes.conf[i])
                        
                        # Get bounding box coordinates
                        box = detection.boxes.xyxy[i].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = box
                        
                        # Color based on whether it's a valid class (green) or not (red)
                        if assigned_label > 0:
                            color = (0, 255, 0)  # Green for valid classes
                        else:
                            color = (0, 0, 255)  # Red for ignored classes
                        
                        # Draw bounding box
                        cv2.rectangle(depth_viz_color, (x1, y1), (x2, y2), color, 2)
                        
                        # Get depth stats in the bounding box region
                        roi_depth = depth_scaled[y1:y2, x1:x2]
                        valid_depth = roi_depth[(roi_depth > 0.1) & (roi_depth < 50)]
                        if len(valid_depth) > 0:
                            depth_text = f"{cls_str}: {np.median(valid_depth):.1f}m"
                        else:
                            depth_text = f"{cls_str}: NO DEPTH"
                        
                        # Draw label with depth info
                        cv2.putText(depth_viz_color, depth_text, (x1, y1-5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            depth_with_det_msg = bridge.cv2_to_imgmsg(depth_viz_color, "bgr8")
            depth_with_det_msg.header = rgb.header
            self.depth_with_detections_pub.publish(depth_with_det_msg)
        except Exception as e:
            rospy.logerr("Error publishing depth with detections: %s", e)
        
        # Create a grid of pixel coordinates
        # ------------- old pinhole cam 3d reconstruction --------------------------
        # u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        # u = u.astype(np.float32)
        # v = v.astype(np.float32)
        # d = depth_img.flatten()
        # d = d * self.depth_scale
        # x = (u.flatten() - self.color_cx) * d / self.color_fx
        # y = (v.flatten() - self.color_cy) * d / self.color_fy
        # z = d

        # x_pt = x.reshape(-1, depth_img.shape[1])
        # y_pt = y.reshape(-1, depth_img.shape[1])
        # z_pt = z.reshape(-1, depth_img.shape[1])
        # x_pt = x_pt[..., None]
        # y_pt = y_pt[..., None]
        # z_pt = z_pt[..., None]
        # print("dafhbArtgn",x,y,z)
        # points = np.concatenate((x_pt, y_pt, z_pt), axis=2)
        # ------------- old pinhole cam 3d reconstruction --------------------------




        # ------------- New lidar depth 3D reconstruction --------------------------
# --- START: Cylindrical 3D Reconstruction Code ---
        # NOTE: Ensure self.W_img, self.f_c, and self.y_c_0 are defined.

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u_flat = u.flatten().astype(np.float32)
        v_flat = v.flatten().astype(np.float32)

        # The depth value 'd' is the radial distance in the XZ plane
        radial_distance = depth_img.flatten() * self.depth_scale

        # Calculate the horizontal angle (azimuth) from the u-coordinate
        azimuth = (u_flat * 2 * np.pi / self.W_img) - np.pi

        # Calculate X, Y, Z coordinates using the inverse cylindrical projection.
        # Note: We calculate flattened 1D arrays first, like in your original code.
        x = radial_distance * np.sin(azimuth)
        z = radial_distance * np.cos(azimuth)  # Z is now dependent on the angle
        y = 1*(v_flat - self.y_c_0) * radial_distance / self.f_c

        # Reshape X, Y, Z back into the image dimensions
        x_pt = x.reshape(depth_img.shape)
        y_pt = y.reshape(depth_img.shape)
        z_pt = z.reshape(depth_img.shape)

        # Add a new axis to each for stacking into a (H, W, 3) point cloud
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]

        # Concatenate to create the final point cloud
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)
        # --- END: Cylindrical 3D Reconstruction Code ---

        # ------------- New lidar depth 3D reconstruction --------------------------

        # DEBUG: Check depth/XYZ values for labeled regions
        labeled_mask = label > 0
        if np.sum(labeled_mask) > 0:
            labeled_depths = depth_img[labeled_mask] * self.depth_scale
            labeled_xyz = points[labeled_mask]
            labeled_ranges = np.linalg.norm(labeled_xyz, axis=1)
            
            # Count valid vs invalid depth
            valid_depth_mask = labeled_depths < 50.0  # reasonable max range
            invalid_depth_count = np.sum(~valid_depth_mask)
            valid_depth_count = np.sum(valid_depth_mask)
            
            print(f"[DEBUG DETECT] Labeled pixels: {np.sum(labeled_mask)}")
            print(f"[DEBUG DETECT] Valid depth (<50m): {valid_depth_count}, Invalid depth (>=50m): {invalid_depth_count}")
            if valid_depth_count > 0:
                valid_depths = labeled_depths[valid_depth_mask]
                valid_ranges = labeled_ranges[valid_depth_mask]
                print(f"[DEBUG DETECT] Valid depth range: min={np.min(valid_depths):.3f}, max={np.max(valid_depths):.3f}, median={np.median(valid_depths):.3f}")
                print(f"[DEBUG DETECT] Valid XYZ range: min={np.min(valid_ranges):.3f}, max={np.max(valid_ranges):.3f}, median={np.median(valid_ranges):.3f}")

        # 5. Stack labels, id, conf
        
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)
        self.make_fields()
        
        # 6. publish point cloud
        sync_pc_odom_msg = syncPcOdom()
        sync_pc_odom_msg.header = Header()
        sync_pc_odom_msg.header.stamp = odom.header.stamp
        sync_pc_odom_msg.header.frame_id = "camera"

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = odom.header.stamp
        pc_msg.header.frame_id = "camera"
        rospy.logwarn_throttle(5, 'Hard coding segmented point cloud frame_id to \"camera\"')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        rospy.loginfo_throttle(3, "Published synced point cloud odom msg")
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)

                

    def rgb_aligned_depth_callback(self, rgb, aligned_depth):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            rospy.loginfo_throttle(3, f"Time elapsed since last depth rgb callback is: {(rospy.Time.now() - self.prev_time).to_sec()}")
            rospy.loginfo_throttle(3, f"Skipping current depth image to get desired rate of {self.desired_rate}")
            return
        else:
            self.prev_time = rospy.Time.now()

        try:
            # Convert ROS image message to OpenCV image
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img, show=False, device=self.device, verbose=False)

        # 2. open img_size * 2 array save class and id
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])
        
        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections is of size 1
            # detections[0] contains everything
            # masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                # print(detection.masks.masks)     # masks, (N, H, W)
                # print(detection.masks.segments)  # bounding coordinates of masks, List[segment] * N
                # print(detection.boxes.conf)   # confidence score, (N, 1)
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()

                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])
        
        # Publish annotated image with bounding boxes overlay
        # The plot() method works even when there are no detections (returns original image)
        try:
            annotated_img = detections[0].plot()
            annotated_img_msg = bridge.cv2_to_imgmsg(annotated_img, "bgr8")
            annotated_img_msg.header = rgb.header
            self.detection_image_pub.publish(annotated_img_msg)
        except Exception as e:
            rospy.logerr("Error publishing detection image: %s", e)
        
        # 4. go through depth, project them to 3D
        # print(depth_img.shape)

        # check depth values
        # unique, counts = np.unique(depth_img, return_counts=True)
        # print(dict(zip(unique, counts)))

        # Convert the depth image to a 3D point cloud
        # rows, cols = depth_img.shape[:2]
        # for u in range(cols):
        #     for v in range(rows):
        #         d = depth_img[v, u]
        #         if not np.isnan(d) and not np.isinf(d):
        #             x = (u - self.depth_cx) * d / self.depth_fx
        #             y = (v - self.depth_cy) * d / self.depth_fy
        #             z = d
        #             pc_pos[v, u, :] = [x, y, z]
        #         else:
        #             pc_pos[v, u, :] = [-1, -1, -1]

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = aligned_depth.header.stamp
        rospy.logwarn_throttle(5, 'Hard coding segmented point cloud frame_id to \"camera\"')
        pc_msg.header.frame_id = "camera"
        # print('hard coding seg pc frame_id to body')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        self.pc_pub_.publish(pc_msg)
        rospy.loginfo_throttle(3, "Published synced point cloud odom msg")

    def get_cls_label(self, cls_str):
        if cls_str in self.cls.keys():
            return self.cls[cls_str]
        else:
            return 0#4

    def run(self):
        # Spin the ROS node
        rospy.loginfo("Semantic detection node init.")
        rospy.spin()

    def make_fields(self):
        # manually add fiels by Ian
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'id'
        field.count = 1
        field.offset = 16
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'confidence'
        field.count = 1
        field.offset = 20
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields


if __name__ == '__main__':
    node = sem_detection()
    node.run()
