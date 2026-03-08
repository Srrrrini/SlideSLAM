#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SlideSlamWs="${SLIDESLAM_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"  # workspace root
SlideSlamCodeDir="${SLIDESLAM_CODE_DIR:-$SlideSlamWs/src/SLIDE_SLAM}"
BAGS_DIR="${BAGS_DIR:-$SlideSlamWs/bags}"                        # bags / data directory
CONTAINER_NAME="slideslam_ros"
IMAGE_NAME="xurobotics/slide-slam:latest"

if [[ ! -d "$SlideSlamCodeDir" ]]; then
  echo "Expected repository at: $SlideSlamWs/src/SLIDE_SLAM"
  echo "Set SLIDESLAM_WS if your workspace is in a different path."
  exit 1
fi

mkdir -p "$BAGS_DIR"
# Ensure LiDAR deps are present (old-style flow, no container branching).
if [[ ! -d "$SlideSlamWs/src/ouster_example/.git" ]]; then
  rm -rf "$SlideSlamWs/src/ouster_example"
  git clone https://github.com/ouster-lidar/ouster_example.git "$SlideSlamWs/src/ouster_example"
  git -C "$SlideSlamWs/src/ouster_example" checkout 43107a1
fi

if [[ ! -d "$SlideSlamWs/src/ouster_decoder/.git" ]]; then
  rm -rf "$SlideSlamWs/src/ouster_decoder"
  git clone https://github.com/KumarRobotics/ouster_decoder.git "$SlideSlamWs/src/ouster_decoder"
  git -C "$SlideSlamWs/src/ouster_decoder" checkout d66b52d
fi

# Clear stale package cache to avoid ouster_ros CMake issues.
cleanup_workspace_path() {
  local target="$1"
  rm -rf "$target" 2>/dev/null || true
  if [[ -e "$target" ]]; then
    # Root-owned artifacts from prior container runs; remove via container as root.
    docker run --rm \
      --volume "$SlideSlamWs:/opt/slideslam_docker_ws" \
      "$IMAGE_NAME" \
      bash -lc "rm -rf '/opt/slideslam_docker_ws/${target#$SlideSlamWs/}'"
  fi
}

cleanup_workspace_path "$SlideSlamWs/build/ouster_ros"
cleanup_workspace_path "$SlideSlamWs/devel/.private/ouster_ros"
cleanup_workspace_path "$SlideSlamWs/logs/ouster_ros"

# Allow RViz/X11 apps from container.
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

xhost +local:root # for the lazy and reckless
docker run -it \
    --name="$CONTAINER_NAME" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/opt/slideslam_docker_ws" \
    --env="DISPLAY=${DISPLAY:-}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH:-}" \
    --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
    --volume="$BAGS_DIR:/opt/bags" \
    "$IMAGE_NAME" \
    bash -lc "set -e; source /opt/ros/noetic/setup.bash; cd /opt/slideslam_docker_ws; catkin build -DCMAKE_BUILD_TYPE=Release; source /opt/slideslam_docker_ws/devel/setup.bash; echo 'SlideSLAM workspace is built and sourced.'; exec bash"