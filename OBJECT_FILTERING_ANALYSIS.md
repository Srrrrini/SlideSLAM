# Object Filtering Analysis & Fixes

## Executive Summary

Analysis of log files reveals that **valid objects are being discarded at the initial cuboid fitting stage** due to overly strict thresholds. The primary issues are:

1. **Initial length threshold too strict** (0.1m) - rejecting objects before they can accumulate enough points
2. **Depth percentile filter too aggressive** (0-75%) - removing too many valid points
3. **Insufficient point accumulation** - objects need multiple frames to build up enough points

---

## Key Findings from Log Analysis

### 1. Initial Cuboid Fitting Rejections

**Problem**: Objects are rejected at the very first filtering stage with messages like:
```
-> REJECTED at initial cuboid fitting: length 0.000 <= 0.100
-> REJECTED at initial cuboid fitting: length 0.042 <= 0.100
```

**Root Cause**: The `fit_cuboid_dim_thresh: 0.1` parameter is too strict. Objects need to have a computed length of at least 0.1m in their **first frame** to pass, but:
- Many detections have very few points (1-11 points) after depth filtering
- With few points, the computed length is often 0.0m or very small (< 0.1m)
- Objects need multiple frames to accumulate enough points to pass this threshold

**Example from logs**:
- Line 144: Only 1 point passed depth filtering → length = 0.0m → REJECTED
- Line 195: 7 points passed → length = 0.042m → REJECTED (0.042 < 0.1)

### 2. Depth Percentile Filter Too Aggressive

**Problem**: The depth percentile filter (0-75%) removes too many valid points:
```
depth_percentile: (0, 75)
cur_depth_values [1.32177842 1.32240927 1.33704472]
depth_thres_low 1.321778416633606
depth_thres_high 1.3297269940376282
valid_pts_mask [False  True False]  # Only 1 out of 3 points passed!
```

**Root Cause**: 
- Using 0-75% percentile means keeping only points between the minimum and 75th percentile
- For objects with depth variation, this can remove 25% of the points
- With few points to begin with, this leaves insufficient points for cuboid fitting

### 3. Class-Specific Issues

**Problem**: Many classes show "skipping this class" messages:
```
i 1  # chair
[False False False ... False False False]
skipping this class
i 2  # table
[False False False ... False False False]
skipping this class
i 3  # tv
[False False False ... False False False]
skipping this class
```

**Root Cause**: 
- Tables and TVs are either not being detected by YOLO, or
- They're being filtered out before reaching the class-specific processing stage
- The range threshold (40m) or other early filters may be removing them

### 4. Successful Case (What Works)

When objects DO pass, they have:
- Enough points after depth filtering (7+ points)
- Length > 0.1m threshold (0.221m in successful case)
- Confidence > 0.3 (0.638 in successful case)
- Final geometric check passes (length 0.222m within [0.2, 6.0] range)

---

## Proposed Fixes

### Fix 1: Lower Initial Cuboid Dimension Threshold ⭐ **HIGH PRIORITY**

**Current**: `fit_cuboid_dim_thresh: 0.1` (10cm)

**Proposed**: `fit_cuboid_dim_thresh: 0.05` (5cm) or `0.03` (3cm)

**Rationale**: 
- Allows objects with fewer accumulated points to pass initial filtering
- Objects can then accumulate more points over multiple frames
- The final geometric filter (chair: 0.2-6.0m) will still catch objects that are too small
- This is a "soft" initial filter vs. the "hard" final filter

**File**: `src/SLIDE_SLAM/frontend/scan2shape/scan2shape_launch/config/process_cloud_node_indoor_params.yaml`
```yaml
# Change line 27 from:
fit_cuboid_dim_thresh: 0.1
# To:
fit_cuboid_dim_thresh: 0.05  # or 0.03 for even more lenient
```

### Fix 2: Relax Depth Percentile Filter ⭐ **HIGH PRIORITY**

**Current**: `depth_percentile_upper: 75` (keeps 0-75% of depth values)

**Proposed**: `depth_percentile_upper: 90` or `95`

**Rationale**:
- Keeps more valid points from each detection
- Reduces the chance of having too few points after filtering
- Still removes outliers (top 5-10% of depth values)

**File**: `src/SLIDE_SLAM/frontend/scan2shape/scan2shape_launch/config/process_cloud_node_indoor_params.yaml`
```yaml
# Change line 15 from:
depth_percentile_upper: 75
# To:
depth_percentile_upper: 90  # or 95 for even more points
```

### Fix 3: Lower Confidence Threshold (Optional)

**Current**: `confidence_threshold: 0.3`

**Proposed**: `confidence_threshold: 0.25` or `0.2`

**Rationale**:
- YOLO detections with confidence 0.25-0.3 may still be valid
- Lower threshold allows more detections to pass initial filtering
- Objects can be refined later through tracking

**Note**: Only do this if you're still missing valid objects after Fixes 1 & 2.

**File**: `src/SLIDE_SLAM/frontend/scan2shape/scan2shape_launch/config/process_cloud_node_indoor_params.yaml`
```yaml
# Change line 5 from:
confidence_threshold: 0.3
# To:
confidence_threshold: 0.25
```

### Fix 4: Add Minimum Point Count Check (Code Change)

**Problem**: Objects with only 1-2 points are being processed, leading to length=0

**Proposed**: Add a minimum point count check before computing cuboid dimensions

**File**: `src/SLIDE_SLAM/frontend/scan2shape/scan2shape_launch/script/cuboid_utils_indoor.py`

In `fit_cuboid_indoor()`, after depth filtering:
```python
if num_valid_pts < 5:  # Require at least 5 points
    rospy.logwarn("-> REJECTED: insufficient points (%d < 5) for cuboid fitting", num_valid_pts)
    continue
```

This prevents processing objects with too few points that will inevitably fail.

### Fix 5: Investigate Table/TV Detection

**Problem**: Tables and TVs show "skipping this class" - no points found

**Investigation Steps**:
1. Check if YOLO is detecting tables/TVs (check `pane_1_0.log` for "dining table" or "tv" detections)
2. Check if range threshold (40m) is filtering them out
3. Check if class mapping in YOLO output matches the expected class names

**Potential Fix**: Adjust `valid_range_threshold` if tables/TVs are far away, or check YOLO class name mapping.

---

## Recommended Implementation Order

1. **Fix 1** (Lower `fit_cuboid_dim_thresh` to 0.05) - **Do this first**
2. **Fix 2** (Increase `depth_percentile_upper` to 90) - **Do this second**
3. **Fix 4** (Add minimum point count check) - **Code change, test after Fixes 1 & 2**
4. **Fix 3** (Lower confidence threshold) - **Only if still missing objects**
5. **Fix 5** (Investigate table/TV) - **Separate investigation**

---

## Expected Results After Fixes

- **More objects passing initial cuboid fitting**: Objects with 0.05-0.1m length will now pass
- **More points available**: 90% depth percentile keeps more valid points
- **Better accumulation**: Objects can accumulate points over multiple frames before being rejected
- **Final filter still active**: The class-specific length/height cutoffs (chair: 0.2-6.0m) will still filter out objects that are too small

---

## Testing Recommendations

1. Run the system with Fixes 1 & 2 applied
2. Monitor `pane_1_3.log` for:
   - Fewer "REJECTED at initial cuboid fitting" messages
   - More objects passing to tracking stage
   - More "ACCEPTED as valid [class] cuboid" messages
3. Check RViz to see if more objects appear
4. If still missing objects, apply Fix 3 and Fix 4

---

## Current Parameter Summary

| Parameter | Current Value | Recommended Value | Impact |
|-----------|--------------|-------------------|--------|
| `fit_cuboid_dim_thresh` | 0.1m | 0.05m | ⭐⭐⭐ High - Allows more objects to pass initial filter |
| `depth_percentile_upper` | 75% | 90% | ⭐⭐⭐ High - Keeps more valid points |
| `confidence_threshold` | 0.3 | 0.25 | ⭐⭐ Medium - Allows lower confidence detections |
| `valid_range_threshold` | 40.0m | 40.0m | ⭐ Low - Keep as is unless tables/TVs are far |

---

## Notes

- The successful chair detection (line 251-272) shows the pipeline CAN work when objects have enough points
- The issue is that most detections don't have enough points in the first frame to pass the 0.1m threshold
- Lowering the initial threshold allows objects to "grow" through tracking before final filtering

