# Topic Mapping Analysis: SLIDE_SLAM ↔ map_manager

## Executive Summary

To merge SLIDE_SLAM with map_manager, we need to bridge object pose topics from SLIDE_SLAM format to map_manager's expected format. **No changes to map_manager are allowed** - we must publish on map_manager's topic names from SLIDE_SLAM.

---

## Map Manager Topics (What It Expects)

### Map Manager Subscribes To:

1. **`/semantic_objects`** (map_manager::semanticObjArrayMsg)
   - **Message Type**: `map_manager/semanticObjArrayMsg`
   - **Contains**: Array of semantic objects with position, size, label_id
   - **Purpose**: Semantic object poses for occupancy map
   - **Location**: `map_manager/msg/semanticObjArrayMsg.msg`
   - **Note**: Map manager also **publishes** this topic

2. **`/odom_ugv`** or **`/odom_uav`** (nav_msgs/Odometry)
   - **Purpose**: Robot odometry for occupancy mapping
   - **Currently published by**: SLIDE_SLAM (via sync nodes)
   - **Status**: ✅ Already compatible

3. **`/point_cloud_ugv`** or **`/point_cloud_uav`** (sensor_msgs/PointCloud2)
   - **Purpose**: Point cloud for occupancy mapping
   - **Status**: Needs mapping from SLIDE_SLAM topics

4. **`/cloud_registered`** (sensor_msgs/PointCloud2)
   - **Purpose**: Dense registered point cloud
   - **Status**: Needs mapping from SLIDE_SLAM topics

5. **`/yolo/detection`** or **`/bbox_ugv`** or **`/bbox_uav`** (vision_msgs/Detection2DArray)
   - **Purpose**: Raw 2D detections
   - **Status**: Optional (if using semantic mapping)

6. **`/depth_ugv`** or **`/depth_uav`** (sensor_msgs/Image)
   - **Purpose**: Depth images for occupancy mapping
   - **Status**: Needs mapping if using depth-based occupancy

### Map Manager Publishes:

1. **`/semantic_objects`** (map_manager::semanticObjArrayMsg)
   - **Note**: Map manager can subscribe to its own published topic
   - **Status**: We can publish here from SLIDE_SLAM

2. **`/shared_map`** (map_manager::sharedVoxels)
   - **Purpose**: Shared occupancy map voxels

3. **`/robot_states`** (map_manager::robotStates)
   - **Purpose**: Robot state information

---

## SLIDE_SLAM Topics (What It Currently Publishes)

### Object Pose Topics:

1. **`/robot0/chair_cuboids`** (visualization_msgs/MarkerArray)
   - **Message Type**: `visualization_msgs/MarkerArray`
   - **Contains**: Object poses as RViz markers in **world frame**
   - **Location**: Published by `process_cloud_node.py`
   - **Fields**: Each marker has:
     - `ns`: object class name (e.g., "chair")
     - `pose.position`: x, y, z in world frame
     - `pose.orientation`: quaternion
     - `scale`: dimensions (x, y, z)
     - `id`: object instance ID

2. **`/robot0/chair_cuboids_body`** (visualization_msgs/MarkerArray)
   - **Message Type**: `visualization_msgs/MarkerArray`
   - **Contains**: Object poses in **body frame**
   - **Status**: Similar to above but in robot body frame

3. **`/robot0/semantic_meas_sync_odom`** (sloam_msgs/SemanticMeasSyncOdom)
   - **Message Type**: `sloam_msgs/SemanticMeasSyncOdom`
   - **Contains**: Synced semantic measurements with odometry
   - **Fields**: `ellipsoid_factors[]` with pose and scale

4. **`/robot0/cuboid_measurements`** (sloam_msgs/SemanticMeasSyncOdom)
   - **Message Type**: `sloam_msgs/SemanticMeasSyncOdom`
   - **Status**: Alternative topic for cuboid measurements

### Point Cloud Topics:

1. **`/robot0/sem_detection_segmented_point_cloud`** (sensor_msgs/PointCloud2)
   - **Purpose**: Segmented point cloud from YOLO

2. **`/robot0/filtered_semantic_segmentation`** (sensor_msgs/PointCloud2)
   - **Purpose**: Filtered segmented point cloud

3. **`/robot0/pc_instance_segmentation_accumulated`** (sensor_msgs/PointCloud2)
   - **Purpose**: Accumulated instance segmentation point cloud

### Odometry Topics:

1. **`/odom_ugv`** (nav_msgs/Odometry)
   - **Status**: ✅ Already compatible with map_manager

---

## Message Format Comparison

### Map Manager Message Format:

```msg
# map_manager/msg/semanticObjArrayMsg.msg
semanticObjMsg[] semanticObjs

# map_manager/msg/semanticObjMsg.msg
int32 label_id
bool height_covered
float64[] position      # [x, y, z]
float64[] size          # [length, width, height]
bool[] view_angles
geometry_msgs/Point[] voxels
```

### SLIDE_SLAM Message Format:

```msg
# visualization_msgs/MarkerArray (from /robot0/chair_cuboids)
visualization_msgs/Marker[] markers
  - string ns              # "chair", "table", etc.
  - int32 id               # instance ID
  - geometry_msgs/Pose pose
    - geometry_msgs/Point position
    - geometry_msgs/Quaternion orientation
  - geometry_msgs/Vector3 scale  # dimensions
```

---

## Required Topic Mappings

### Priority 1: Object Poses (Most Important)

| SLIDE_SLAM Topic | Map Manager Topic | Action Required |
|-----------------|-------------------|-----------------|
| `/robot0/chair_cuboids` → | `/semantic_objects` | **Create bridge node** to convert MarkerArray → semanticObjArrayMsg |

**Conversion Logic:**
- Subscribe to `/robot0/chair_cuboids` (MarkerArray)
- Convert each marker to `semanticObjMsg`:
  - `label_id`: Map class name to integer (chair=1, table=2, etc.)
  - `position`: Extract from `marker.pose.position` → `[x, y, z]`
  - `size`: Extract from `marker.scale` → `[x, y, z]`
  - `height_covered`: Set to false (or determine from height)
  - `view_angles`: Empty array (optional)
  - `voxels`: Empty array (optional)
- Publish as `semanticObjArrayMsg` to `/semantic_objects`

### Priority 2: Point Clouds (If needed)

| SLIDE_SLAM Topic | Map Manager Topic | Action Required |
|-----------------|-------------------|-----------------|
| `/robot0/pc_instance_segmentation_accumulated` → | `/point_cloud_ugv` | Simple topic remap (or relay node) |
| `/robot0/filtered_semantic_segmentation` → | `/cloud_registered` | Simple topic remap (or relay node) |

### Priority 3: Odometry (Already Compatible)

| SLIDE_SLAM Topic | Map Manager Topic | Status |
|-----------------|-------------------|--------|
| `/odom_ugv` | `/odom_ugv` | ✅ Already matches |

---

## Implementation Plan

### Step 1: Create Bridge Node

Create a new ROS node in SLIDE_SLAM that:
1. **Subscribes** to `/robot0/chair_cuboids` (visualization_msgs/MarkerArray)
2. **Converts** MarkerArray → map_manager::semanticObjArrayMsg
3. **Publishes** to `/semantic_objects` (map_manager::semanticObjArrayMsg)

**Node Name**: `slide_slam_to_map_manager_bridge` or `object_pose_bridge`

**Location**: `src/SLIDE_SLAM/frontend/object_modeller/script/` or create new package

### Step 2: Class Label Mapping

Need to map SLIDE_SLAM class names to map_manager label_ids:
- "chair" → 1
- "table" → 2
- "tv" → 3
- (Add more as needed)

### Step 3: Topic Remapping (Optional)

For point clouds, can use ROS topic remapping in launch files instead of a node.

---

## File Structure

```
src/SLIDE_SLAM/
├── frontend/
│   └── object_modeller/
│       └── script/
│           └── slide_slam_to_map_manager_bridge.py  # NEW FILE
│       └── launch/
│           └── map_manager_bridge.launch            # NEW LAUNCH FILE
```

---

## Key Constraints

1. ✅ **Cannot modify map_manager** - all changes must be in SLIDE_SLAM
2. ✅ **Must publish on map_manager topic names** - `/semantic_objects` specifically
3. ✅ **Must not break existing SLIDE_SLAM pipeline** - bridge node is additive
4. ✅ **Message format must match exactly** - use map_manager message types

---

## Next Steps

1. **Create the bridge node** (`slide_slam_to_map_manager_bridge.py`)
2. **Add launch file** to start the bridge node
3. **Update tmux script** to launch the bridge node
4. **Test** that map_manager receives object poses correctly

---

## Configuration Files

### Map Manager Config:
- `map_manager/cfg/occupancy_map_param.yaml`:
  - `odom_topic: /odom_ugv`
  - `raw_detection_topic: /yolo/detection` (optional)
  - `dense_cloud_topic: /cloud_registered` (optional)

### SLIDE_SLAM Config:
- Already publishes `/robot0/chair_cuboids`
- Already publishes `/odom_ugv`

---

## Verification

After implementing the bridge:

1. **Check topic existence**:
   ```bash
   rostopic list | grep semantic_objects
   rostopic echo /semantic_objects -n 1
   ```

2. **Check message format**:
   ```bash
   rostopic type /semantic_objects
   # Should output: map_manager/semanticObjArrayMsg
   ```

3. **Monitor in map_manager**:
   - Map manager should automatically subscribe to `/semantic_objects`
   - Check map_manager logs for object reception

