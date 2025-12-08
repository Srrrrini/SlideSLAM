#include "lidar_cam_calibrater/RegisterRGBD.h"
#include <tf/transform_broadcaster.h>

RegisterRGBD::RegisterRGBD(ros::NodeHandle &node){
    image_transport::ImageTransport it(node);
    
    // Get topic names from parameters with defaults
    std::string spot_image_topic, test_sub_image_topic, registered_color_topic, registered_depth_topic, spot_odom_topic, odom_ugv_topic, rgb_ugv_topic, depth_ugv1_topic;
    node.param<std::string>("spot_image_topic", spot_image_topic, "/spot_image");
    node.param<std::string>("test_sub_image_topic", test_sub_image_topic, "/test_sub_image");
    node.param<std::string>("registered_color_topic", registered_color_topic, "/camera_front/processed_rgb");
    node.param<std::string>("registered_depth_topic", registered_depth_topic, "/robot0/camera/aligned_depth_to_color/image_raw");
    node.param<std::string>("spot_odom_topic", spot_odom_topic, "/Odometry");
    node.param<std::string>("odom_ugv_topic", odom_ugv_topic, "/odom_ugv");
    node.param<std::string>("rgb_ugv_topic", rgb_ugv_topic, "/robot0/camera/color/image_raw");
    node.param<std::string>("depth_ugv1_topic", depth_ugv1_topic, "/robot0/camera/depth/image_rect_raw");
    
    image_sub = it.subscribe(spot_image_topic, 1, &RegisterRGBD::imageCallback, this);
    image_pub = it.advertise(test_sub_image_topic, 1);
    registered_color_pub = it.advertise(registered_color_topic, 1);
    registered_depth_pub = it.advertise(registered_depth_topic, 1);
    depth_ugv1_pub = it.advertise(depth_ugv1_topic, 1);
    
    // Original spot image republisher
    spot_image_sub = it.subscribe(spot_image_topic, 1, &RegisterRGBD::spotImageCallback, this);
    rgb_ugv_pub = it.advertise(rgb_ugv_topic, 1);
    
    // Odom subscriber and publishers (publish to multiple topics for map_manager compatibility)
    odom_sub = node.subscribe(spot_odom_topic, 100, &RegisterRGBD::odomCallback, this);
    odom_pub = node.advertise<nav_msgs::Odometry>(odom_ugv_topic, 100);
    odom_uav_pub = node.advertise<nav_msgs::Odometry>("/odom_uav", 100);
    spot_odom_pub = node.advertise<nav_msgs::Odometry>("/spot/odom", 100);
    spot_odometry_pub = node.advertise<nav_msgs::Odometry>("/spot_Odometry", 100);

    lidar_pointcloud_sub.subscribe(node, "/os_node/points",100); 
    stitched_image_sub.subscribe(node, "/spot_image",100);
    
    sync_.reset(new Sync(MySyncPolicy(50), lidar_pointcloud_sub, stitched_image_sub));
    sync_->registerCallback(boost::bind(&RegisterRGBD::SyncLidar2RGB, this, _1, _2));
}

void RegisterRGBD::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // Convert the ROS Image message to a CV2 Image
    // ROS_INFO("[register rgbd]: img callback");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Get the CV image
    cv::Mat cv_image = cv_ptr->image;

    // Check image dimensions
    int width = cv_image.cols;
    int height = cv_image.rows;
    for (size_t i=0 ; i< height ; i++){
        for (size_t j=0 ; j<width ; j++){
            // if (j>1200){
            // if (i>450 || i<280){
            //     // cv_image.at<float>(i,j) = 0.;
            //     cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // Set pixel to black
            // }
            if (350 < i && i < 370){
                cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
            }

            if (395 < i && i < 405){
                cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    // Publish the modified image
    
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    image_pub.publish(output_msg);
}

// void RegisterRGBD::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
//     // Simply republish the odom message to the new topic
//     odom_pub.publish(msg);
// }

void RegisterRGBD::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy incoming message from faster-lio
    // Faster-lio publishes: camera_init (world frame) -> body (robot body frame)
    nav_msgs::Odometry odom_msg = *msg;

    // Make odom_ugv an alias for camera_init (fixed world frame)
    // Publish identity transform: camera_init -> odom_ugv
    static tf::TransformBroadcaster br;
    
    // Identity transform: odom_ugv = camera_init (fixed world frame)
    tf::Transform identity_transform;
    identity_transform.setOrigin(tf::Vector3(0, 0, 0));
    identity_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(identity_transform,
        msg->header.stamp,
        "camera_init",   // parent frame (world frame from faster-lio)
        "odom_ugv"       // child frame (alias for camera_init)
    ));

    // The incoming odom message is: camera_init -> body
    // We need to compute: camera_init -> camera = (camera_init -> body) * (body -> camera)
    // body -> camera is a static rotation transform (no translation)
    // Rotation matrix from body to camera (from utils.py):
    // R = [[0, 0, 1],
    //      [-1, 0, 0],
    //      [0, -1, 0]]
    // This is the inverse of camera -> body transform
    
    // Get the transform from camera_init to body from the odom message
    tf::Vector3 body_pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    tf::Quaternion body_quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    // Create transform from camera_init to body
    tf::Transform camera_init_to_body;
    camera_init_to_body.setOrigin(body_pos);
    camera_init_to_body.setRotation(body_quat);
    
    // Static transform from body to camera (rotation only, no translation)
    // This matches the transform in scan2shape_launch/script/utils.py line 322-325
    tf::Matrix3x3 rot_body_to_cam;
    rot_body_to_cam[0][0] = 0.0;  rot_body_to_cam[0][1] = 0.0;  rot_body_to_cam[0][2] = 1.0;
    rot_body_to_cam[1][0] = -1.0; rot_body_to_cam[1][1] = 0.0;  rot_body_to_cam[1][2] = 0.0;
    rot_body_to_cam[2][0] = 0.0;  rot_body_to_cam[2][1] = -1.0; rot_body_to_cam[2][2] = 0.0;
    tf::Transform body_to_camera;
    body_to_camera.setOrigin(tf::Vector3(0, 0, 0));
    body_to_camera.setBasis(rot_body_to_cam);
    
    // Publish transform: odom_ugv -> body (for SLOAM and robot visualization)
    // Since odom_ugv = camera_init, this is directly from faster-lio
    br.sendTransform(tf::StampedTransform(camera_init_to_body,
        msg->header.stamp,
        "odom_ugv",   // parent frame (alias for camera_init)
        "body"        // child frame (robot body frame for SLOAM)
    ));
    
    // Compute camera_init -> camera = (camera_init -> body) * (body -> camera)
    // This is needed for semantic processing (scan2shape)
    tf::Transform camera_init_to_camera = camera_init_to_body * body_to_camera;
    
    // Publish transform: odom_ugv -> camera (for semantic processing)
    br.sendTransform(tf::StampedTransform(camera_init_to_camera,
        msg->header.stamp,
        "odom_ugv",   // parent frame (alias for camera_init)
        "camera"      // child frame (camera frame for semantic processing)
    ));

    // Publish odometry message with body frame (for SLOAM)
    // SLOAM expects odometry to represent body pose in world frame
    odom_msg.header.frame_id = "odom_ugv";     // parent frame (world frame)
    odom_msg.child_frame_id = "body";          // child frame (robot body frame)
    
    // The pose in the message already represents body pose (from faster-lio)
    // No need to transform - it's already camera_init -> body
    // Just update frame IDs in the message header
    
    // Publish the updated odometry message to all required topics
    odom_pub.publish(odom_msg);           // /odom_ugv
    odom_uav_pub.publish(odom_msg);       // /odom_uav
    spot_odom_pub.publish(odom_msg);      // /spot/odom
    spot_odometry_pub.publish(odom_msg);  // /spot_Odometry
}




void RegisterRGBD::spotImageCallback(const sensor_msgs::ImageConstPtr& msg){
    // Simply republish the original spot image without any processing
    rgb_ugv_pub.publish(msg);
}

void RegisterRGBD::SyncLidar2RGB( const sensor_msgs::PointCloud2::ConstPtr& lidar_pointcloud_msg,
                                 const sensor_msgs::ImageConstPtr& stiched_image_msg)
{
    // measure running time 
    auto start_time = std::chrono::high_resolution_clock::now();


    // ROS_INFO("get sync");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(stiched_image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    
    // create a depth image, and fill points into the image
    cv::Mat depth_image = cv::Mat::ones(image.rows, image.cols, CV_16UC1) * 65534;
    // cv::Mat depth_image = cv::Mat::ones(height, width, CV_16UC1) * 65535;

    
    H_img = stiched_image_msg->height;
    W_img = stiched_image_msg->width;
    // this->pixel2Lidar.clear();
    // this->pixel2Lidar.resize(H_img*W_img, std::vector<Eigen::Vector3d>());
    ROS_INFO("Cylindrical Params Being Used: W_img=%.2f, f_y=%.4f, y_0=%.2f", W_img, f_y, y_0);
    sensor_msgs::PointCloud2ConstIterator<float> lidar_iter(*lidar_pointcloud_msg, "x");
    double max_z = 0.;
    for ( ;  lidar_iter != lidar_iter.end(); ++lidar_iter ) 
    {
        tf::Point lidar_point(lidar_iter[0],lidar_iter[1],lidar_iter[2]); 
        tf::Point lidar_point_2_cam = lidar_point;//transform lidar points from their own frames of refernce to cylindrical image frame 
        lidar_point_2_cam[2] += 0.3;
        if (lidar_point_2_cam[2] > max_z){
            max_z = lidar_point_2_cam[2];
        }
        std::vector<int> lidar_pixels = GetPixelCordinates(lidar_point_2_cam);
        
        // printf("1111111111111111/n");
        if(lidar_pixels[0] < W_img && lidar_pixels[1] < H_img && lidar_pixels[0] >= 0 && lidar_pixels[1] >= 0)
        {
            double dist = std::sqrt(lidar_point_2_cam[0]*lidar_point_2_cam[0] + lidar_point_2_cam[1]*lidar_point_2_cam[1] + lidar_point_2_cam[2]*lidar_point_2_cam[2]);
            double dist2CamPlan = std::sqrt(lidar_point_2_cam[0]*lidar_point_2_cam[0] + lidar_point_2_cam[1]*lidar_point_2_cam[1]);
            cv::Vec3b color = distanceToColor(lidar_point_2_cam[2]+1.0, 10.0);
            // if (lidar_point_2_cam[2] > 2.0){
            //     continue;
            // }
            // Assign the color to the image pixel
            image.at<cv::Vec3b>(lidar_pixels[1], lidar_pixels[0]) = color;

            // make depth image
            // only keep the closest point
            if (depth_image.at<ushort>(lidar_pixels[1], lidar_pixels[0]) > static_cast<uint16_t>(dist2CamPlan*1000)){
                depth_image.at<uint16_t>(lidar_pixels[1], lidar_pixels[0]) = static_cast<uint16_t>(dist2CamPlan*1000);
            }

            // map image pixel to lidar point
            // pixel2Lidar[lidar_pixels[0] + lidar_pixels[1]*W_img].push_back(Eigen::Vector3d(lidar_point_2_cam[0], lidar_point_2_cam[1], lidar_point_2_cam[2]));
        }
            
    
    }

    cv_ptr->image = image;
    sensor_msgs::ImagePtr depth_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth_image).toImageMsg();
    depth_msg_ptr->header.stamp = stiched_image_msg->header.stamp;
    
    if(cv_ptr and depth_msg_ptr)
    {
        registered_color_pub.publish(cv_ptr->toImageMsg());
        registered_depth_pub.publish(depth_msg_ptr);
        depth_ugv1_pub.publish(depth_msg_ptr);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
}

std::vector<int> RegisterRGBD::GetPixelCordinates(const tf::Point &point)
{
    std::vector<int> pixels;
    pixels.resize(2);
    double X,Y,Z;
    X = point[0]; Y = point[1]; Z = point[2];
    pixels[0] = W_img * (-atan2(Y,X) + M_PI) / (2* M_PI); 
    pixels[1] = f_y*(-Z/(std::sqrt(X*X + Y*Y))) + y_0;
    return pixels;
}


cv::Vec3b RegisterRGBD::distanceToColor(double dist, double max_dist) {
    double normalized = dist / max_dist;
    if (normalized > 1.0) {
        normalized = 1.0;
    }
    normalized = std::max(0.0, std::min(1.0, normalized)); // Clamp the value between 0 and 1

    // Map normalized distance to a hue value (0-180 for OpenCV)
    int hue = static_cast<int>(normalized * 180);
    
    // Convert HSV to BGR
    cv::Mat hsv(1, 1, CV_8UC3, cv::Vec3b(hue, 255, 255)); // Full saturation and value
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

    return bgr.at<cv::Vec3b>(0, 0);
}

// cv::Vec3b RegisterRGBD::heightToColor(double hieght, double max_) {
//     double normalized = dist / max_dist;
//     if (normalized > 1.0) {
//         normalized = 1.0;
//     }
//     normalized = std::max(0.0, std::min(1.0, normalized)); // Clamp the value between 0 and 1

//     // Map normalized distance to a hue value (0-180 for OpenCV)
//     int hue = static_cast<int>(normalized * 180);
    
//     // Convert HSV to BGR
//     cv::Mat hsv(1, 1, CV_8UC3, cv::Vec3b(hue, 255, 255)); // Full saturation and value
//     cv::Mat bgr;
//     cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

//     return bgr.at<cv::Vec3b>(0, 0);
// }