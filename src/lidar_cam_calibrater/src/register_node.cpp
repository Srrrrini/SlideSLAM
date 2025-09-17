#include "lidar_cam_calibrater/RegisterRGBD.h"
#include "lidar_cam_calibrater/testRGBD.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "register_node");
    ros::NodeHandle node;
    RegisterRGBD register_rgbd(node);
    // testRGBD test_rgbd(node);
    ros::Rate loop_rate(10);
    while (ros::ok())
	{
        //std::cout<<"while loop\n";
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0; 
}