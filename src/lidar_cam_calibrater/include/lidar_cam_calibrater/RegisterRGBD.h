#ifndef REGISTER_RGBD_H
#define REGISTER_RGBD_H

#define M_PI 3.14159265358979323846  /* pi */
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

class RegisterRGBD
{
public:
    RegisterRGBD(){}
    ~RegisterRGBD(){}
    RegisterRGBD(ros::NodeHandle &node);
    
    // std::vector<std::vector<Eigen::Vector3d>> pixel2Lidar;

protected:

private:
    void SyncLidar2RGB(const sensor_msgs::PointCloud2::ConstPtr& lidar_piontcloud, const sensor_msgs::Image::ConstPtr& stitched_image);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void spotImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    std::vector<int> GetPixelCordinates(const tf::Point &point);
    cv::Vec3b distanceToColor(double dist, double max_dist);

    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_transport::Publisher registered_color_pub;
    image_transport::Publisher registered_depth_pub;
    image_transport::Publisher depth_ugv1_pub;
    
    // Original spot image republisher
    image_transport::Subscriber spot_image_sub;
    image_transport::Publisher rgb_ugv_pub;
    
    // Odom subscriber and publisher
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2>lidar_pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> stitched_image_sub;
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    int W_img, H_img;

    // f_y resized from Spot-CAM specification
    float f_y = 219.4734627;
    float y_0 = 375;

    // manually tuned parameters
    // float f_y = 180;//median of calibrated cameras' focal lengths
    // float y_0 = 375;//median of calibrated cameras' offsets

    // ros::Publisher depth_encoded_pub; 

};

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>

// class RegisterRGBD
// {
// public:
//     RegisterRGBD(ros::NodeHandle &node);
//     ~RegisterRGBD(){}

// private:
//     void imageCallback(const sensor_msgs::ImageConstPtr& msg);

//     image_transport::Subscriber image_sub;
//     image_transport::Publisher image_pub;

//     float f_y = 731.6612593757163; // median of calibrated cameras' focal lengths
//     float y_0 = 573.8239536818945; // median of calibrated cameras' offsets
// };
#endif 
