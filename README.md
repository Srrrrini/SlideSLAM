# SlideSLAM Docker Workspace (Extended)

This repository is work built on top of the original SlideSLAM project:

- [KumarRobotics/SLIDE_SLAM](https://github.com/KumarRobotics/SLIDE_SLAM)

## Citation

This project builds on SlideSLAM. 

Liu et al., *Slideslam: Sparse, lightweight, decentralized metric-semantic slam for multi-robot navigation*, arXiv preprint arXiv:2406.17249, 2024.

<details>
<summary>BibTeX</summary>

```bibtex
@article{liu2024slideslam,
  title={Slideslam: Sparse, lightweight, decentralized metric-semantic slam for multi-robot navigation},
  author={Liu, Xu and Lei, Jiuzhou and Prabhu, Ankit and Tao, Yuezhan and Spasojevic, Igor and Chaudhari, Pratik and Atanasov, Nikolay and Kumar, Vijay},
  journal={arXiv preprint arXiv:2406.17249},
  year={2024}
}
```

</details>

## Quick Start

Clone this repository with submodules (required):

```bash
git clone --recurse-submodules git@github.com:Srrrrini/SlideSLAM.git
cd SlideSLAM
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

Launch the SlideSLAM docker environment:

```bash
chmod +x src/SLIDE_SLAM/run_slide_slam_docker.sh
./src/SLIDE_SLAM/run_slide_slam_docker.sh
```

This script starts (or reuses) the Docker container, builds the workspace with `catkin build`, sources it, and then drops you into an interactive shell.

After entering the Docker shell, make sure the bag `824indoor_sync.bag` exists in `bags/indoor` (for example, check with `ls bags/indoor/824indoor_sync.bag`), then run:

```bash
roscd multi_robot_utils_launch/script/
./tmux_single_indoor_robot.sh
```

## Loop Closure

Loop closure is controlled by a launch parameter and several tuning knobs:

### Enabling / Disabling

In `src/SLIDE_SLAM/backend/sloam/launch/single_robot_sloam_test.launch`:

```xml
<param name="turn_off_intra_loop_closure" value="false"/>  <!-- false = enabled -->
```

### Place Recognition Parameters

In `src/SLIDE_SLAM/backend/sloam/params/sloam.yaml` under `place_recognition`:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `min_num_inliers` | 10 | Minimum matched objects to accept a loop closure. Lowered to 2 for indoor scenes with few objects. |
| `min_num_map_objects_to_start` | 100 | Minimum landmarks in the map before attempting place recognition. Set to 5 for indoor. |
| `compute_budget_sec` | 5.0 | Time budget (seconds) for the SlideMatch search. Set to 10.0 for indoor. |
| `match_threshold_position` | 0.5 | XY distance threshold for position matching. Set to 0.75 for indoor. |

### Loop Closure Noise Model

In `src/SLIDE_SLAM/backend/sloam/src/factorgraph/graph.cpp` (requires recompilation after changes):

```cpp
Vector6 noise_model_pose_vec = Vector6::Ones() * 0.00001;  // base sigma = 1e-5
noise_model_closure =
    noiseModel::Diagonal::Sigmas(noise_model_pose_vec * 0.01);  // sigma = 1e-7
```

The multiplier (`0.01`) controls how much the optimizer trusts the loop closure relative to odometry. Smaller = tighter constraint (more trust in closure). With few inliers (2-3), the computed transform can be noisy, so increasing this multiplier (e.g. `10.0`, `100.0`) softens the correction.

**Note:** When loop closure is active and the robot enters a revisit region, SLOAM clears all semantic measurements in that area to avoid map corruption. With too few inliers this can hurt more than it helps.

### Recompiling After C++ Changes

```bash
# Inside the Docker container
source /opt/ros/noetic/setup.bash
cd /opt/slideslam_docker_ws
catkin build sloam
```

## References

For detailed usage and dataset/demo instructions, refer to:

- `src/SLIDE_SLAM/README.md`
