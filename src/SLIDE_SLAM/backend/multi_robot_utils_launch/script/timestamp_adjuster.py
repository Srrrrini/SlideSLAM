# #!/usr/bin/env python

# import rospy
# import rosbag
# from std_msgs.msg import String
# from sensor_msgs.msg import Image  # Example for image messages, add others as needed

# def modify_bag_timestamps(input_bag, output_bag):
#     # Open the input bag file and the output bag file
#     with rosbag.Bag(input_bag, 'r') as in_bag, rosbag.Bag(output_bag, 'w') as out_bag:
#         first_time = None  # Initialize variable to store the first timestamp
        
#         # Loop over each message in the bag file
#         for topic, msg, t in in_bag.read_messages():
#             # Set first_time to the timestamp of the first message
#             if first_time is None:
#                 first_time = t
                
#             # Calculate the new timestamp by maintaining the original time difference
#             time_diff = t - first_time
#             new_timestamp = rospy.Time.now() + time_diff  # Adjust relative to current time
            
#             # Update timestamp for messages with header (e.g., sensor_msgs/Image)
#             if hasattr(msg, 'header'):
#                 msg.header.stamp = new_timestamp
            
#             # Write the message to the new bag file with the adjusted timestamp
#             out_bag.write(topic, msg, new_timestamp)
            
#             # If the topic is "camera_front/depth_ugv", duplicate the message to "depth_ugv1"
#             if topic == '/camera_front/depth_ugv':
#                 # Duplicate the message to the new topic "depth_ugv1"
#                 out_bag.write('depth_ugv1', msg, new_timestamp)

# if __name__ == '__main__':
#     # Initialize the ROS node
#     rospy.init_node('timestamp_adjuster')

#     # Set the input and output .bag file names
#     input_bag = '/opt/bags/indoor/test.bag'  # Modify with the path to your input .bag file
#     output_bag = '/opt/bags/indoor/test1.bag'  # Modify with the desired output .bag file

#     # Call the function to modify timestamps and write to a new bag file
#     try:
#         modify_bag_timestamps(input_bag, output_bag)
#         rospy.loginfo(f"Successfully wrote modified messages to {output_bag}")
#     except Exception as e:
#         rospy.logerr(f"Error processing bag file: {e}")

#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Image  # Example for image messages, add others as needed

def duplicate_depth_topic(input_bag, output_bag):
    # Open the input bag file and the output bag file
    with rosbag.Bag(input_bag, 'r') as in_bag, rosbag.Bag(output_bag, 'w') as out_bag:
        # Loop over each message in the bag file
        for topic, msg, t in in_bag.read_messages():
            # Write the original message to the new bag file
            out_bag.write(topic, msg, t)

            # If the topic is "/depth_ugv", duplicate it to "/depth_ugv1"
            if topic == '/depth_ugv':
                out_bag.write('/depth_ugv1', msg, t)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('depth_topic_duplicator')

    # Set the input and output .bag file names
    input_bag = '/opt/bags/indoor/sim_ugv.bag'  # Modify with the path to your input .bag file
    output_bag = '/opt/bags/indoor/sim_ugv_duplicated.bag'  # Modify with the desired output .bag file

    # Call the function to duplicate the depth topic
    try:
        duplicate_depth_topic(input_bag, output_bag)
        rospy.loginfo(f"Successfully wrote modified messages to {output_bag}")
    except Exception as e:
        rospy.logerr(f"Error processing bag file: {e}")





