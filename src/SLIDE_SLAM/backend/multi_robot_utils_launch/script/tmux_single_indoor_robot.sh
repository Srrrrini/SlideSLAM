#!/bin/bash


SESSION_NAME=slide_slam_nodes
BAG_PLAY_RATE=0.5
# BAG_DIR='/home/sam/bags/vems-slam-bags/all_slide_slam_public_demos/forests'
# BAG_DIR='/opt/bags/vems-slam-bags/all_slide_slam_public_demos/forests'
# BAG_DIR='/home/sam/bags/vems-slam-bags/all_slide_slam_public_demos/indoor'
BAG_DIR='/opt/bags/indoor'

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

SETUP_ROS_STRING="export ROS_MASTER_URI=http://localhost:11311"

# Create log directory (overwrite each run)
LOG_DIR="/opt/slideslam_docker_ws/tmux_logs"
mkdir -p $LOG_DIR
# Clear old logs
rm -f $LOG_DIR/*.log
echo "==========================================="
echo "Logging tmux panes to: $LOG_DIR"
echo "==========================================="

# Make mouse useful in copy mode
tmux setw -g mouse on


tmux new-window -t $SESSION_NAME -n "Main"

# Split horizontally → left/right columns
tmux split-window -h -t $SESSION_NAME:1.0

# Left column: create 3 panes (1.0–1.2)
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME:1.0

# Right column: create 3 panes (1.3–1.5)
tmux select-pane -t $SESSION_NAME:1.1
tmux split-window -v -t $SESSION_NAME:1.1
tmux split-window -v -t $SESSION_NAME:1.1
tmux split-window -v -t $SESSION_NAME:1.1

tmux select-layout -t $SESSION_NAME:1 tiled

# Start logging for all panes in window 1 with descriptive names
tmux pipe-pane -t $SESSION_NAME:1.0 -o "cat > $LOG_DIR/yolo_detection.log"
tmux pipe-pane -t $SESSION_NAME:1.1 -o "cat > $LOG_DIR/sync_semantic.log"
tmux pipe-pane -t $SESSION_NAME:1.2 -o "cat > $LOG_DIR/sloam.log"
tmux pipe-pane -t $SESSION_NAME:1.3 -o "cat > $LOG_DIR/process_cloud_node.log"
tmux pipe-pane -t $SESSION_NAME:1.4 -o "cat > $LOG_DIR/flio.log"
tmux pipe-pane -t $SESSION_NAME:1.5 -o "cat > $LOG_DIR/pane_1_5.log"

# Launch your 6 processes
tmux send-keys -t $SESSION_NAME:1.0 "$SETUP_ROS_STRING; sleep 2; roslaunch object_modeller rgb_segmentation_f250.launch" Enter
tmux send-keys -t $SESSION_NAME:1.1 "$SETUP_ROS_STRING; sleep 2; roslaunch object_modeller sync_semantic_measurements.launch robot_name:=robot0 odom_topic:=/odom_ugv" Enter
tmux send-keys -t $SESSION_NAME:1.2 "$SETUP_ROS_STRING; sleep 2; roslaunch sloam single_robot_sloam_test.launch enable_rviz:=true" Enter
tmux send-keys -t $SESSION_NAME:1.3 "$SETUP_ROS_STRING; sleep 2; roslaunch scan2shape_launch process_cloud_node_rgbd_indoor_with_ns.launch odom_topic:=/odom_ugv robot_name:=robot0" Enter
tmux send-keys -t $SESSION_NAME:1.4 "$SETUP_ROS_STRING; sleep 2; roslaunch scan2shape_launch run_flio_with_driver.launch" Enter

# --- SECOND WINDOW: 3 panes (added bridge node) ---
tmux new-window -t $SESSION_NAME -n "Extra"
tmux split-window -v -t $SESSION_NAME:2.0
tmux split-window -v -t $SESSION_NAME:2.0
tmux select-layout -t $SESSION_NAME:2 even-vertical

# Start logging for panes in window 2 with descriptive names
tmux pipe-pane -t $SESSION_NAME:2.0 -o "cat > $LOG_DIR/rosbag_play.log"
tmux pipe-pane -t $SESSION_NAME:2.1 -o "cat > $LOG_DIR/register_node.log"
tmux pipe-pane -t $SESSION_NAME:2.2 -o "cat > $LOG_DIR/pane_2_2.log"

# Launch 3 processes
tmux send-keys -t $SESSION_NAME:2.0 "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR; rosbag play 824indoor_sync.bag --clock -r $BAG_PLAY_RATE -s 115 --topics /spot_image /ouster/imu /ouster/points ouster/imu:=/os_node/imu /ouster/points:=/os_node/points" Enter
# tmux send-keys -t $SESSION_NAME:2.0 "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR; rosbag play 824indoor_sync.bag --clock -r $BAG_PLAY_RATE -s 105 --topics /spot/odom /ouster/points /spot_image /spot/odom:=/Odometry /ouster/points:=/os_node/points" Enter

# tmux send-keys -t $SESSION_NAME:2.1 "$SETUP_ROS_STRING; sleep 2; roslaunch sloam single_robot_sloam_test_LiDAR.launch enable_rviz:=true" Enter
tmux send-keys -t $SESSION_NAME:2.1 "$SETUP_ROS_STRING; sleep 2; rosrun lidar_cam_calibrater register_node" Enter

# Bridge node to convert SLIDE_SLAM object poses to map_manager format
# Includes: odom relay (publishes /odom_ugv to /odom_ugv and /odom_uav) + object pose bridge
# Note: Requires map_manager message types to be built in SLIDE_SLAM workspace
tmux send-keys -t $SESSION_NAME:2.2 "$SETUP_ROS_STRING; sleep 5; roslaunch object_modeller map_manager_bridge.launch" Enter

# Optional: return focus to main window
tmux select-window -t $SESSION_NAME:1

# Print log locations
echo ""
echo "=========================================="
echo "Tmux session '$SESSION_NAME' started!"
echo "Logs at: $LOG_DIR/"
echo ""
echo "Key logs:"
echo "  yolo_detection.log      - YOLO detections"
echo "  process_cloud_node.log  - Point cloud processing"
echo "=========================================="

# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR; rosparam set /use_sim_time true; rosbag play ugv_yolo_duplicated.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /odom_ugv /depth_ugv /depth_ugv1 /rgb_ugv /depth_ugv:=/robot0/camera/aligned_depth_to_color/image_raw /rgb_ugv:=/robot0/camera/color/image_raw /depth_ugv1:=/robot0/camera/depth/image_rect_raw" Enter
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play test1.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /odom_ugv depth_ugv1 /camera_front/rgb_ugv /camera_front/depth_ugv depth_ugv1:=/robot0/camera/aligned_depth_to_color/image_raw /camera_front/rgb_ugv:=/robot0/camera/color/image_raw /camera_front/depth_ugv:=/robot0/camera/depth/image_rect_raw" Enter
# Add lidar_cam_calibrater node 
# tmux split-window -h -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR; rosparam set /use_sim_time true;rosrun topic_tools relay /robot0/camera/aligned_depth_to_color/image_raw /robot0/camera/depth/image_rect_raw & sleep 0; rosbag play sim_ugv.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /odom_ugv /camera_front/depth_ugv /camera_front/rgb_ugv /camera_front/depth_ugv:=/robot0/camera/aligned_depth_to_color/image_raw /camera_front/rgb_ugv:=/robot0/camera/color/image_raw" Enter
# tmux select-pane -t $SESSION_NAME:1.4
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play test1.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /odom_ugv depth_ugv1 /camera_front/rgb_ugv /camera_front/depth_ugv depth_ugv1:=/robot0/camera/aligned_depth_to_color/image_raw /camera_front/rgb_ugv:=/robot0/camera/color/image_raw /camera_front/depth_ugv:=/robot0/camera/depth/image_rect_raw" Enter
# default code
# tmux select-pane -t $SESSION_NAME:1.0
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch object_modeller rgb_segmentation_f250.launch" Enter
# tmux select-pane -t $SESSION_NAME:1.1
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch object_modeller sync_semantic_measurements.launch robot_name:=robot0 odom_topic:=/dragonfly67/quadrotor_ukf/control_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.2
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch sloam single_robot_sloam_test.launch enable_rviz:=true" Enter
# tmux select-pane -t $SESSION_NAME:1.3
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch scan2shape_launch process_cloud_node_rgbd_indoor_with_ns.launch odom_topic:=/dragonfly67/quadrotor_ukf/control_odom robot_name:=robot0" Enter
# tmux select-pane -t $SESSION_NAME:1.4
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play indoor-f250-*.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /dragonfly67/quadrotor_ukf/control_odom /camera/aligned_depth_to_color/image_raw /camera/color/image_raw /camera/depth/image_rect_raw /camera/aligned_depth_to_color/image_raw:=/robot0/camera/aligned_depth_to_color/image_raw /camera/color/image_raw:=/robot0/camera/color/image_raw /camera/depth/image_rect_raw:=/robot0/camera/depth/image_rect_raw" Enter




""'

roscd multi_robot_utils_launch/script
./tmux_single_indoor_robot.sh

""'




# original
# tmux select-pane -t $SESSION_NAME:1.4
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play indoor-f250-*.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /dragonfly67/quadrotor_ukf/control_odom /camera/aligned_depth_to_color/image_raw /camera/color/image_raw /camera/depth/image_rect_raw /camera/aligned_depth_to_color/image_raw:=/robot0/camera/aligned_depth_to_color/image_raw /camera/color/image_raw:=/robot0/camera/color/image_raw /camera/depth/image_rect_raw:=/robot0/camera/depth/image_rect_raw" Enter


# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot4*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot4/odom /robot0/semantic_meas_sync_odom:=/robot4/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.5
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot5*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot5/odom /robot0/semantic_meas_sync_odom:=/robot5/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.6
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot6*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot6/odom /robot0/semantic_meas_sync_odom:=/robot6/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.7
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot7*.bag --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot7/odom /robot0/semantic_meas_sync_odom:=/robot7/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.8
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING;" Enter
# tmux select-layout -t $SESSION_NAME tiled


# Add window for roscore
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux select-pane -t $SESSION_NAME:2.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; rosparam set /use_sim_time true" Enter


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"


tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
