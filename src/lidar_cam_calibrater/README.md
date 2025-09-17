## Description
ROS package used to project 3D lidar point clouds onto a panaromic RGB image. Used for ouster_ldar to bosdon dynamic SPOT CAM system.
The mathematic model is similar to what described [here](http://download.cs.stanford.edu/downloads/jrdb/Sensor_setup_JRDB.pdf)


## Usage 
First, make sure image and lidar point cloud topic are available.
Then, 
Run the registration node
```
rosrun lidar_cam_calibrater register_node
```

