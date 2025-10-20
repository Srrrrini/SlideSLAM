#include "lidar_cam_calibrater/RegisterRGBD.h"

RegisterRGBD::RegisterRGBD(ros::NodeHandle &node){
    image_transport::ImageTransport it(node);
    
    // Get topic names from parameters with defaults
    std::string spot_image_topic, test_sub_image_topic, registered_color_topic, registered_depth_topic, spot_odom_topic, odom_ugv_topic, rgb_ugv_topic, depth_ugv1_topic;
    node.param<std::string>("spot_image_topic", spot_image_topic, "/spot_image");
    node.param<std::string>("test_sub_image_topic", test_sub_image_topic, "/test_sub_image");
    node.param<std::string>("registered_color_topic", registered_color_topic, "/camera_front/processed_rgb");
    node.param<std::string>("registered_depth_topic", registered_depth_topic, "/robot0/camera/aligned_depth_to_color/image_raw");
    node.param<std::string>("spot_odom_topic", spot_odom_topic, "/spot/odom");
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
    
    // Odom subscriber and publisher
    odom_sub = node.subscribe(spot_odom_topic, 100, &RegisterRGBD::odomCallback, this);
    odom_pub = node.advertise<nav_msgs::Odometry>(odom_ugv_topic, 100);

    lidar_pointcloud_sub.subscribe(node, "/ouster/points",100); 
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

void RegisterRGBD::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Simply republish the odom message to the new topic
    odom_pub.publish(msg);
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