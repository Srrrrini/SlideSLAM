# Object Pose Access Guide

When valid objects (chairs/tables) are detected and displayed in RViz, their pose estimates are published on multiple ROS topics. Here's how to access them:

## 📍 **Primary Topics (Direct Access)**

### 1. **`/robot0/chair_cuboids`** (visualization_msgs/MarkerArray)
**Frame**: `odom_ugv` (world frame)  
**Publisher**: `process_cloud_node`  
**Contains**: Full 3D pose + dimensions for each object

**Access via ROS command:**
```bash
rostopic echo /robot0/chair_cuboids
```

**Access via Python:**
```python
import rospy
from visualization_msgs.msg import MarkerArray

def callback(msg):
    for marker in msg.markers:
        # Object class name (chair/table)
        class_name = marker.ns
        
        # 3D Position (centroid)
        x = marker.pose.position.x
        y = marker.pose.position.y
        z = marker.pose.position.z
        
        # Orientation (quaternion)
        qx = marker.pose.orientation.x
        qy = marker.pose.orientation.y
        qz = marker.pose.orientation.z
        qw = marker.pose.orientation.w
        
        # Dimensions (length, width, height)
        length = marker.scale.x
        width = marker.scale.y
        height = marker.scale.z
        
        # Unique ID
        obj_id = marker.id
        
        print(f"Object {obj_id} ({class_name}): pos=({x:.2f}, {y:.2f}, {z:.2f}), "
              f"dim=({length:.2f}, {width:.2f}, {height:.2f})")

rospy.init_node('object_pose_listener')
rospy.Subscriber('/robot0/chair_cuboids', MarkerArray, callback)
rospy.spin()
```

### 2. **`/robot0/chair_cuboids_body`** (visualization_msgs/MarkerArray)
**Frame**: `body` (robot body frame)  
**Publisher**: `process_cloud_node`  
**Contains**: Same as above, but in robot body frame

```bash
rostopic echo /robot0/chair_cuboids_body
```

---

## 🔄 **Synced Topics (For SLAM Backend)**

### 3. **`/robot0/cuboid_measurements`** (sloam_msgs/StampedRvizMarkerArray)
**Publisher**: `cylinder_plane_modeller`  
**Contains**: Cuboids with timestamps, ready for odometry synchronization

```bash
rostopic echo /robot0/cuboid_measurements
```

### 4. **`/robot0/semantic_meas_sync_odom_raw`** (sloam_msgs/SemanticMeasSyncOdom)
**Publisher**: `sync_all` (merge_synced_measurements)  
**Contains**: Cuboids synchronized with odometry (used by SLAM backend)

**Message Structure:**
```
header:
  stamp: <timestamp>
  frame_id: <frame>
odometry: <nav_msgs/Odometry>
cuboid_factors: <ROSCube[]>
  - pose: <geometry_msgs/Pose>
      position: {x, y, z}
      orientation: {x, y, z, w}
    scale: [length, width, height]
    semantic_label: <int>
```

**Access via Python:**
```python
import rospy
from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import ROSCube

def callback(msg):
    # Odometry at time of detection
    odom = msg.odometry
    
    # All detected objects
    for cuboid in msg.cuboid_factors:
        # Position
        pos = cuboid.pose.position
        print(f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        
        # Orientation (quaternion)
        ori = cuboid.pose.orientation
        print(f"Orientation: ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})")
        
        # Dimensions
        print(f"Dimensions: {cuboid.scale}")
        
        # Semantic label (1=chair, 2=table, etc.)
        print(f"Semantic label: {cuboid.semantic_label}")

rospy.init_node('semantic_pose_listener')
rospy.Subscriber('/robot0/semantic_meas_sync_odom_raw', SemanticMeasSyncOdom, callback)
rospy.spin()
```

---

## 📊 **Quick Reference: Topic Hierarchy**

```
YOLO Detection
    ↓
process_cloud_node
    ├─ /robot0/chair_cuboids (world frame) ← EASIEST TO ACCESS
    └─ /robot0/chair_cuboids_body (body frame)
        ↓
cylinder_plane_modeller
    └─ /robot0/cuboid_measurements
        ↓
sync_cuboid_odom
    └─ /robot0/semantic_meas_sync_odom_raw (synced with odom)
        ↓
merge_synced_measurements
    └─ /robot0/semantic_meas_sync_odom_raw (merged)
        ↓
SLOAM Backend (consumes for SLAM)
```

---

## 🛠️ **Practical Examples**

### Example 1: List all detected objects
```bash
rostopic echo /robot0/chair_cuboids -n 1 | grep -A 10 "ns:"
```

### Example 2: Get latest object positions
```bash
rostopic echo /robot0/chair_cuboids -n 1 | grep -E "(ns:|position:|scale:)"
```

### Example 3: Monitor object count
```bash
rostopic echo /robot0/chair_cuboids -n 1 | grep "markers:" | wc -l
```

### Example 4: Python script to save poses to file
```python
#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
import json
from datetime import datetime

poses = []

def callback(msg):
    timestamp = datetime.now().isoformat()
    for marker in msg.markers:
        pose_data = {
            'timestamp': timestamp,
            'id': marker.id,
            'class': marker.ns,
            'position': {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'z': marker.pose.position.z
            },
            'orientation': {
                'x': marker.pose.orientation.x,
                'y': marker.pose.orientation.y,
                'z': marker.pose.orientation.z,
                'w': marker.pose.orientation.w
            },
            'dimensions': {
                'length': marker.scale.x,
                'width': marker.scale.y,
                'height': marker.scale.z
            }
        }
        poses.append(pose_data)
        print(f"Saved pose for {marker.ns} ID {marker.id}")

rospy.init_node('pose_recorder')
rospy.Subscriber('/robot0/chair_cuboids', MarkerArray, callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    with open('object_poses.json', 'w') as f:
        json.dump(poses, f, indent=2)
    print(f"\nSaved {len(poses)} poses to object_poses.json")
```

---

## 📝 **Message Field Details**

### MarkerArray Message Structure:
```yaml
markers:
  - header:
      stamp: <time>
      frame_id: "odom_ugv"  # or "body" for body frame
    ns: "chair"  # or "table" - object class name
    id: 0  # Unique object ID
    type: MESH_RESOURCE  # or SPHERE if no mesh model
    action: ADD
    pose:
      position: {x: float, y: float, z: float}  # Centroid position
      orientation: {x: float, y: float, z: float, w: float}  # Quaternion
    scale: {x: float, y: float, z: float}  # Dimensions (length, width, height)
    color: {r: float, g: float, b: float, a: float}
    mesh_resource: "package://sloam/resource/chair.stl"  # If CAD model exists
```

---

## 🎯 **Recommended Approach**

**For simple access**: Use `/robot0/chair_cuboids` - it's the easiest and contains all pose information.

**For SLAM integration**: Use `/robot0/semantic_meas_sync_odom_raw` - it's synchronized with odometry and used by the SLAM backend.

**For robot-relative poses**: Use `/robot0/chair_cuboids_body` - positions are relative to the robot's current pose.

---

## 🔍 **Debugging: Check if objects are being published**

```bash
# Check topic exists and has data
rostopic list | grep cuboid

# Check message rate
rostopic hz /robot0/chair_cuboids

# Check message type
rostopic type /robot0/chair_cuboids

# View latest message
rostopic echo /robot0/chair_cuboids -n 1
```

