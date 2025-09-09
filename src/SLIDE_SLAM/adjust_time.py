import rosbag
import rospy
from std_msgs.msg import Header

# Input and output bag file paths
input_bag_path = '/bags/indoor/test.bag'
output_bag_path = 'adjusted_test.bag'

# Open the input bag
with rosbag.Bag(input_bag_path, 'r') as in_bag, rosbag.Bag(output_bag_path, 'w') as out_bag:
    # Iterate through each message in the input bag
    for topic, msg, t in in_bag.read_messages():
        
        # If the message has a header, adjust the timestamp
        if hasattr(msg, 'header') and msg.header:
            # Find the reference topic (for example, '/camera_front/depth_ugv' as reference)
            if topic == '/odom_ugv':
                # Normalize /odom_ugv timestamp to the simulation time base
                msg.header.stamp = t
            elif topic == '/camera_front/depth_ugv':
                # Normalize /camera_front/depth_ugv timestamp to the simulation time base
                msg.header.stamp = t
            else:
                # For other topics, normalize them to the same time base as well
                msg.header.stamp = t
            
        # Write the adjusted message to the output bag
        out_bag.write(topic, msg, t)

print(f'Adjusted bag saved as {output_bag_path}')
