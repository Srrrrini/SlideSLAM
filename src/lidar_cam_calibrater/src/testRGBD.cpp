#include "lidar_cam_calibrater/testRGBD.h"
testRGBD::testRGBD(ros::NodeHandle &node){
    image_transport::ImageTransport it(node);
    image_sub = it.subscribe("/spot_image", 1, &testRGBD::imageCallback, this);
    image_pub = it.advertise("/test_sub_image", 1);
}

void testRGBD::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // Convert the ROS Image message to a CV2 Image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


    // Get the CV image
    cv::Mat cv_image = cv_ptr->image;

    // Check image dimensions
    int width = cv_image.cols;
    int height = cv_image.rows;

    if (width > 1200) {
        for (int i = 0; i < height; ++i) {
            for (int j = 1200; j < width; ++j) {
                cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // Set pixel to black
            }
        }
    }

    // Publish the modified image
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    image_pub.publish(output_msg);
}


