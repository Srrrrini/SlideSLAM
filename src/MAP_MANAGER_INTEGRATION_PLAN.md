## Map Manager – SLIDE_SLAM Integration Plan

**Entry point you use:** `roslaunch map_manager occupancy_map.launch`  
**Goal:** Use SLIDE_SLAM’s 3D object pose estimates as the source of truth for object centers/sizes in `map_manager`, while keeping `occupancy_map.launch` as the main launch file.

---

## 1. High‑Level Phases

- **Phase 0 – Sanity checks (no code changes)**
- **Phase 1 – Use SLIDE_SLAM poses to override published semantic objects (low risk)**
- **Phase 2 – Initialize new `objectMap`s from SLIDE_SLAM poses (deeper integration, optional)**

All changes below are intended to live in **your fork** of `map_manager` (`Srrrrini/map_manager`, branch `dev`).

---

## 2. Current Pipelines (Summary)

- **Map Manager (single‑robot `occupancy_map.launch`):**
  - Core class: `occMap` in `map_manager/include/map_manager/occupancyMap.h/.cpp`
  - Creates and updates `objectMap` instances (`map_manager/include/map_manager/objectMap.h/.cpp`)
  - Builds `semanticObjects_` and publishes `/semantic_objects` internally via `occMap::publishSemanticObjMsg()`
  - Does **not** currently use external `/semantic_objects` as input.

- **SLIDE_SLAM side (already implemented in your SlideSLAM workspace):**
  - `slide_slam_to_map_manager_bridge.py`  
    - Sub: `/robot0/chair_cuboids` (`visualization_msgs/MarkerArray` in `camera_init`)  
    - TF: transforms to `map` frame  
    - Pub: `/semantic_objects` (`map_manager/semanticObjArrayMsg`) with `label_id`, `position[]` (in `map`), `size[]`, etc.
  - `detection_adapter.py` (**NEW**)
    - Sub: `/sem_detection/pointcloud` (`sensor_msgs/PointCloud2` with semantic labels)
    - Processes per-pixel labels to extract 2D bounding boxes per instance
    - Pub: `/yolo/detection` (`vision_msgs/Detection2DArray`) for map_manager's raw detection pipeline
  - `odom_relay.py`, `point_cloud_relay.py` already handle odom + cloud → `map_manager`.

---

## 3. Phase 0 – Sanity Checks (No Code Changes)

**Files involved (for reference only, no edits yet):**
- `map_manager/launch/occupancy_map.launch`
- `src/SLIDE_SLAM/frontend/object_modeller/launch/map_manager_bridge.launch`
- `src/SLIDE_SLAM/frontend/object_modeller/script/slide_slam_to_map_manager_bridge.py`

**Checks:**
- Run:
  - `roslaunch map_manager occupancy_map.launch`
  - Your SLIDE_SLAM tmux script that launches `map_manager_bridge.launch`
- Verify:
  - `/semantic_objects` exists and contains objects in **`map`** frame.
  - TF tree has a valid chain to `map` (e.g. `camera_init ↔ map`).
  - `map_manager` still works as before if SLIDE_SLAM bridge is not running.

---

## 3.5 Phase 0.5 – Enable SLIDE_SLAM Detection Adapter (DONE)

**Objective:**  
Feed SLIDE_SLAM's YOLO detections into map_manager's raw detection pipeline so that map_manager can generate its own bounding boxes and semantic objects. This is a prerequisite for Phase 1.

**Implementation:**
- Created `detection_adapter.py` in `src/SLIDE_SLAM/frontend/object_modeller/script/`
- Subscribes to `/sem_detection/pointcloud` (SLIDE_SLAM's segmented point cloud with per-pixel labels, ids, confidences)
- Extracts 2D bounding boxes for each detected instance by:
  - Grouping points by instance ID
  - Finding pixel-space min/max coordinates for each instance
  - Computing bounding box center and size
  - Extracting class label and average confidence
- Publishes `vision_msgs/Detection2DArray` to `/yolo/detection`
- Added to `map_manager_bridge.launch`

**Configuration in map_manager:**
- In `map_manager/cfg/occupancy_map_param.yaml`, ensure:
  - `use_semantic: true` (enables semantic processing)
  - `raw_detection_topic: /yolo/detection` (subscribes to adapter output)

**Dependencies:**
- Requires `vision_msgs` ROS package. Install in SLIDE_SLAM Docker container:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-vision-msgs
  ```
- Then rebuild the workspace:
  ```bash
  cd ~/catkin_ws && catkin_make
  ```

**Testing:**
1. Launch SLIDE_SLAM system (including `map_manager_bridge.launch`)
2. Launch `roslaunch map_manager occupancy_map.launch`
3. Verify:
   - `/yolo/detection` topic is being published
   - map_manager displays bounding boxes in RViz
   - Bounding boxes align roughly with detected objects

---

## 4. Phase 1 – Override Published Semantic Objects with SLIDE_SLAM Poses

**Objective:**  
Keep `objectMap` internals unchanged, but **inject SLIDE_SLAM poses right before publishing `/semantic_objects`** so visualization and consumers see SLIDE_SLAM‑aligned centers and sizes.

### 4.1 Files to Modify

- **`map_manager/include/map_manager/occupancyMap.h`**
- **`map_manager/include/map_manager/occupancyMap.cpp`**

### 4.2 Detailed Changes

- **A. Add storage for external semantic objects** (`occupancyMap.h`)

  In the **semantic data** section (around existing `semanticObjects_`):

  ```cpp
  // semantic data
  std::vector<vision_msgs::Detection3D> objectDetections_;
  std::vector<vision_msgs::Detection3D> rawBoxes_;
  std::vector<semanticObject> semanticObjects_;

  // NEW: latest semantic objects from SLIDE_SLAM bridge
  std::vector<semanticObject> externalSemanticObjects_;
  ```

- **B. Add subscriber and callback declaration** (`occupancyMap.h`)

  In the **public** section:

  ```cpp
  // NEW: subscribe to external semantic objects from SLIDE_SLAM bridge
  void externalSemanticObjSubCB(const map_manager::semanticObjArrayMsgConstPtr& objArray);
  ```

  In the **member variables** section (with other subscribers/publishers):

  ```cpp
  // DETECTION / SEMANTIC DATA
  ros::Subscriber rawObjDetectSub_;
  ros::Subscriber registeredMapSub_;
  // ...
  ros::Publisher semanticObjPub_;

  // NEW:
  ros::Subscriber externalSemanticObjSub_;
  ```

- **C. Initialize subscriber in constructor / init** (`occupancyMap.cpp`)

  In the constructor or `initOccMap` function where other subs/pubs are set up:

  ```cpp
  externalSemanticObjSub_ = nh_.subscribe(
      "/semantic_objects",                      // from SLIDE_SLAM bridge
      1,
      &occMap::externalSemanticObjSubCB,
      this);
  ```

- **D. Implement `externalSemanticObjSubCB`** (`occupancyMap.cpp`)

  Near other callbacks:

  ```cpp
  void occMap::externalSemanticObjSubCB(const map_manager::semanticObjArrayMsgConstPtr& objArray)
  {
      std::lock_guard<std::mutex> lock(this->objectMapMutex_); // reuse existing mutex if appropriate

      externalSemanticObjects_.clear();
      for (const auto& incomeObject : objArray->semanticObjs) {
          semanticObject obj;
          obj.label_id = incomeObject.label_id;

          obj.position(0) = incomeObject.position[0];
          obj.position(1) = incomeObject.position[1];
          obj.position(2) = incomeObject.position[2];

          obj.size(0) = incomeObject.size[0];
          obj.size(1) = incomeObject.size[1];
          obj.size(2) = incomeObject.size[2];

          // view_angles from message (0/1); make sure lengths match
          obj.view_angles.clear();
          obj.view_angles.reserve(incomeObject.view_angles.size());
          for (bool v : incomeObject.view_angles) {
              obj.view_angles.push_back(v ? 1 : 0);
          }

          // voxels field in message can be ignored here (kept at default)

          externalSemanticObjects_.push_back(obj);
      }
  }
  ```

- **E. Blend external poses into `publishSemanticObjMsg`** (`occupancyMap.cpp`)

  In `occMap::publishSemanticObjMsg()` where the `/semantic_objects` message is assembled from `this->semanticObjects_`:

  1. For each internal `semanticObject& object : this->semanticObjects_`:
     - Try to find a matching external object:
       - Same `label_id`.
       - Distance between centers < threshold (e.g. 2.0 m).
     - If found, **override**:
       - `object.position` ← external position
       - `object.size` ← external size
       - Optionally OR‑combine `view_angles`.

  2. Then continue existing logic that converts `semanticObjects_` → `semanticObjArrayMsg` and publishes.

  Pseudo‑code inside `publishSemanticObjMsg()`:

  ```cpp
  for (auto& object : this->semanticObjects_) {
      bool matched = false;
      for (const auto& ext : this->externalSemanticObjects_) {
          if (ext.label_id != object.label_id) continue;

          double dist = (ext.position - object.position).norm();
          if (dist < 2.0) { // configurable threshold
              object.position = ext.position;
              object.size     = ext.size;
              // merge view_angles: any 1 in external sets to 1
              if (object.view_angles.size() == ext.view_angles.size()) {
                  for (size_t i = 0; i < object.view_angles.size(); ++i) {
                      if (ext.view_angles[i]) object.view_angles[i] = 1;
                  }
              }
              matched = true;
              break;
          }
      }
  }
  ```

**Result:**  
`/semantic_objects` published by `occupancy_map.launch` will be **aligned to SLIDE_SLAM poses** whenever external data is available, but the underlying `objectMap` voxel grids remain untouched.

---

## 5. Phase 2 – Initialize New ObjectMaps from SLIDE_SLAM Poses (Optional / Deeper Change)

**Objective:**  
When `occMap::updateObjectMapInput()` creates a new `objectMap`, use SLIDE_SLAM’s pose as the initial **center** (and optionally size), instead of only the raw 3D center from depth.

### 5.1 Files to Modify

- **`map_manager/include/map_manager/occupancyMap.cpp`**

### 5.2 Detailed Changes

- **A. Re‑use `externalSemanticObjects_`**

  From Phase 1, `externalSemanticObjects_` already stores latest SLIDE_SLAM semantic objects in **`map`** frame.

- **B. Modify `updateObjectMapInput` new‑object branch**

  In `occMap::updateObjectMapInput(...)` (inside the `if (newObj)` block where `objectMap` is constructed):

  Current code (simplified):

  ```cpp
  Eigen::Vector3d center(
      this->objectDetections_[i].bbox.center.position.x,
      this->objectDetections_[i].bbox.center.position.y,
      this->objectDetections_[i].bbox.center.position.z);

  // ... viewAngleBase computation ...

  std::shared_ptr<mapManager::objectMap> objectMap =
      std::make_shared<mapManager::objectMap>(
          this->objectMapParams_,
          center,
          this->objectDetections_[i].results[0].id,
          viewAngleBase,
          this->sensorManager_);
  ```

  **New behavior:**

  1. Compute detection center as before (`center_detect`).
  2. Search `externalSemanticObjects_` for a matching entry:
     - Same `label_id` (from `results[0].id`).
     - Distance between detection center and external position < threshold (e.g. 2.0 m).
  3. If a match exists:
     - Use external `position` as `center` when constructing `objectMap`.
     - Optionally adapt `objectMapParams_.object_map_size` to external `size` (within caps).
  4. If no match:
     - Fall back to original `center_detect`.

  Pseudo‑code:

  ```cpp
  Eigen::Vector3d center_detect(
      this->objectDetections_[i].bbox.center.position.x,
      this->objectDetections_[i].bbox.center.position.y,
      this->objectDetections_[i].bbox.center.position.z);

  Eigen::Vector3d center = center_detect;
  int label_id = this->objectDetections_[i].results[0].id;

  double bestDist = std::numeric_limits<double>::max();
  for (const auto& ext : this->externalSemanticObjects_) {
      if (ext.label_id != label_id) continue;
      double dist = (ext.position - center_detect).norm();
      if (dist < 2.0 && dist < bestDist) { // configurable
          bestDist = dist;
          center = ext.position;
      }
  }

  // Use 'center' (possibly overridden) for objectMap creation
  std::shared_ptr<mapManager::objectMap> objectMap =
      std::make_shared<mapManager::objectMap>(
          this->objectMapParams_,
          center,
          label_id,
          viewAngleBase,
          this->sensorManager_);
  ```

**Result:**  
New object maps will be centered around SLIDE_SLAM’s pose when it is available and reasonably close to the detection cluster, improving alignment between the voxel map and the SLIDE_SLAM cuboid.

---

## 6. Testing Checklist

**With `roslaunch map_manager occupancy_map.launch` + SLIDE_SLAM bridge running:**

- **After Phase 1:**
  - `/semantic_objects`:
    - Positions (in `map`) closely match SLIDE_SLAM `/robot0/chair_cuboids`.
    - `rostopic echo /semantic_objects` shows reasonable coordinates.
  - RViz (fixed frame `map`):
    - SLIDE_SLAM cuboids and map_manager semantic markers overlap.

- **After Phase 2 (optional):**
  - Object map bounding boxes and occupancy blobs are centered where SLIDE_SLAM cuboids are.
  - Center‑shifting (`recenterMap`) still works but should require fewer corrections.

---

## 7. Design Constraints / Notes

- **Launch file remains `map_manager/launch/occupancy_map.launch`.**  
  No changes required to the way you start map_manager.

- **Soft dependency on SLIDE_SLAM:**
  - If `/semantic_objects` from SLIDE_SLAM bridge is missing, `externalSemanticObjects_` stays empty.
  - Blend and new‑object center logic fall back to original behavior.

- **Map frame consistent:**
  - SLIDE_SLAM bridge already publishes semantic objects in `map` frame.
  - `occMap` also uses `map` as its primary frame, so no extra TF gymnastics should be needed.


