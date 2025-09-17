#ifndef TEST_RGBD_H
#define TEST_RGBD_H

// #define M_PI 3.14159265358979323846  /* pi */
// #include <ros/ros.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <cv_bridge/cv_bridge.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
// #include <math.h>
// #include <image_transport/image_transport.h>

// class testRGBD
// {
// public:
//     testRGBD(){}
//     ~testRGBD(){}
//     testRGBD(ros::NodeHandle &node);
    

// protected:

// private:
//     // void SyncLidar2RGB(const sensor_msgs::PointCloud2::ConstPtr& lidar_piontcloud, const sensor_msgs::Image::ConstPtr& stitched_image);
//     void imageCallback(const sensor_msgs::ImageConstPtr& msg);

//     image_transport::Subscriber image_sub;
//     image_transport::Publisher image_pub;

//     float f_y = 731.6612593757163;//median of calibrated cameras' focal lengths
//     float y_0 = 573.8239536818945;//median of calibrated cameras' offsets
//     // float f_y = 484.352;//median of calibrated cameras' focal lengths
//     // float y_0 = 208.799;//median of calibrated cameras' offsets
//     //float y_0 = 1880;
//     ros::Publisher depth_encoded_pub; 

// };
// #endif 



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class testRGBD
{
public:
    testRGBD(ros::NodeHandle &node);
    ~testRGBD(){}

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    float f_y = 731.6612593757163; // median of calibrated cameras' focal lengths
    float y_0 = 573.8239536818945; // median of calibrated cameras' offsets
};

#endif