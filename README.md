# SlideSLAM Docker Workspace (Extended)

This repository is work built on top of the original SlideSLAM project:

- [KumarRobotics/SLIDE_SLAM](https://github.com/KumarRobotics/SLIDE_SLAM)

## Citation

Original SlideSLAM paper:

```bibtex
@article{liu2024slideslam,
  title={Slideslam: Sparse, lightweight, decentralized metric-semantic slam for multi-robot navigation},
  author={Liu, Xu and Lei, Jiuzhou and Prabhu, Ankit and Tao, Yuezhan and Spasojevic, Igor and Chaudhari, Pratik and Atanasov, Nikolay and Kumar, Vijay},
  journal={arXiv preprint arXiv:2406.17249},
  year={2024}
}
```

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

The script opens a shell inside the container. In that shell, build and source the workspace:

```bash
cd /opt/slideslam_docker_ws
catkin build -DCMAKE_BUILD_TYPE=Release
source /opt/slideslam_docker_ws/devel/setup.bash
```

For detailed usage and dataset/demo instructions, refer to:

- `src/SLIDE_SLAM/README.md`
