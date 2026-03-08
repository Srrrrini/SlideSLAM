#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SlideSlamWs="${SLIDESLAM_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"  # workspace root
BAGS_DIR="${BAGS_DIR:-$SlideSlamWs/bags}"                        # bags / data directory
CONTAINER_NAME="${CONTAINER_NAME:-slideslam_ros}"
IMAGE_NAME="${IMAGE_NAME:-xurobotics/slide-slam:latest}"

if [[ ! -d "$SlideSlamWs/src/SLIDE_SLAM" ]]; then
  echo "Expected repository at: $SlideSlamWs/src/SLIDE_SLAM"
  echo "Set SLIDESLAM_WS if your workspace is in a different path."
  exit 1
fi

mkdir -p "$BAGS_DIR"

# Allow RViz/X11 apps from container.
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

BUILD_AND_SHELL_CMD='set -e
cd /opt/slideslam_docker_ws
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
source /opt/slideslam_docker_ws/devel/setup.bash
echo "SlideSLAM workspace is built and sourced."
exec bash'

if docker ps -a --format "{{.Names}}" | rg -x "$CONTAINER_NAME" >/dev/null; then
  docker start "$CONTAINER_NAME" >/dev/null
  docker exec -it "$CONTAINER_NAME" bash -lc "$BUILD_AND_SHELL_CMD"
else
  docker run -it \
    --name "$CONTAINER_NAME" \
    --net host \
    --privileged \
    --gpus all \
    --workdir /opt/slideslam_docker_ws \
    --env DISPLAY="${DISPLAY:-}" \
    --env QT_X11_NO_MITSHM=1 \
    --volume "$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume "$BAGS_DIR:/opt/bags" \
    "$IMAGE_NAME" \
    bash -lc "$BUILD_AND_SHELL_CMD"
fi