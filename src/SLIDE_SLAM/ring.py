#!/usr/bin/env python

import rosbag
import sys
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def diagnose_point_cloud(input_bag_path, target_topic):
    """
    Reads the first PointCloud2 message on a topic and prints detailed
    diagnostic information about its fields and data types.
    """
    ring_field = PointField(name='ring', offset=0, datatype=PointField.UINT16, count=1)
    INT_DATATYPES = [PointField.INT8, PointField.UINT8, PointField.INT16, PointField.UINT16, PointField.INT32, PointField.UINT32]

    print("--- Starting Diagnostic Script ---")
    print("Bag File: {}".format(input_bag_path))
    print("Target Topic: {}".format(target_topic))
    
    try:
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            print("\nSearching for the first message on topic '{}'...".format(target_topic))
            for topic, msg, t in inbag.read_messages(topics=[target_topic]):
                # --- We only process the VERY FIRST message and then exit ---
                
                print("\n--- Found First Message. Analyzing... ---")
                
                # Define the new fields list that WOULD be created
                new_fields = msg.fields + [ring_field]

                # --- Print detailed info about the FIELDS ---
                print("\n[FIELDS DEFINITION]")
                print("Field # | Name         | Datatype (Enum)")
                print("-----------------------------------------")
                for i, f in enumerate(new_fields):
                    print("Field {:<2} | {:<12} | {}".format(i, f.name, f.datatype))

                # --- Generate and print detailed info about the first POINT ---
                points_iterator = pc2.read_points(msg)
                first_original_point = next(points_iterator)

                final_point_to_pack = []
                for field_definition, value in zip(msg.fields, first_original_point):
                    if field_definition.datatype in INT_DATATYPES:
                        final_point_to_pack.append(int(value))
                    else:
                        final_point_to_pack.append(value)
                
                ring_value = 0 # Ring for the first point is always 0
                final_point_to_pack.append(ring_value)

                print("\n[FIRST POINT DATA TO BE PACKED]")
                print("Value # | Python Type         | Value")
                print("-------------------------------------------------")
                for i, val in enumerate(final_point_to_pack):
                    print("Value {:<2} | {:<19} | {}".format(i, str(type(val)), val))

                print("\n--- Diagnostics finished. Exiting. ---")
                sys.exit(0) # Exit successfully after printing info

    except Exception as e:
        print("\n!!!!!!!!!! AN ERROR OCCURRED !!!!!!!!!!")
        print(e)
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 ring.py <input_bag> [target_topic]")
        sys.exit(1)
        
    input_bag = sys.argv[1]
    target_topic = sys.argv[2] if len(sys.argv) > 2 else '/ouster/points'
    
    diagnose_point_cloud(input_bag, target_topic)