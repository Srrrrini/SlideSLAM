# SlideSLAM Docker Workspace (Extended)

This repository is work built on top of the original SlideSLAM project:

- [KumarRobotics/SLIDE_SLAM](https://github.com/KumarRobotics/SLIDE_SLAM)

## Citation

This project builds on SlideSLAM. If you use this repository, please cite:

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

For detailed usage and dataset/demo instructions, refer to:

- `src/SLIDE_SLAM/README.md`
