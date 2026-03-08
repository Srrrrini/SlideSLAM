#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SlideSlamWs="${SLIDESLAM_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"  # workspace root
BAGS_DIR="${BAGS_DIR:-$SlideSlamWs/bags}"                        # bags / data directory
CONTAINER_NAME="${CONTAINER_NAME:-slideslam_ros}"
IMAGE_NAME="${IMAGE_NAME:-xurobotics/slide-slam:latest}"
INSTALL_LIDAR_DEPS="${SLIDESLAM_INSTALL_LIDAR_DEPS:-1}"
DEPS_CHANGED=0

if [[ ! -d "$SlideSlamWs/src/SLIDE_SLAM" ]]; then
  echo "Expected repository at: $SlideSlamWs/src/SLIDE_SLAM"
  echo "Set SLIDESLAM_WS if your workspace is in a different path."
  exit 1
fi

mkdir -p "$BAGS_DIR"

ensure_repo_at_commit() {
  local target_dir="$1"
  local repo_url="$2"
  local repo_commit="$3"
  local backup_dir

  if [[ -d "$target_dir/.git" ]]; then
    # Ensure expected commit exists locally.
    if ! git -C "$target_dir" checkout "$repo_commit" >/dev/null 2>&1; then
      git -C "$target_dir" fetch --all --tags
      git -C "$target_dir" checkout "$repo_commit"
    fi
    return
  fi

  if [[ -e "$target_dir" ]]; then
    backup_dir="${target_dir}.backup.$(date +%s)"
    echo "Found non-git path at $target_dir. Moving to $backup_dir"
    mv "$target_dir" "$backup_dir"
  fi

  echo "Installing missing dependency: $target_dir"
  git clone "$repo_url" "$target_dir"
  git -C "$target_dir" checkout "$repo_commit"
  DEPS_CHANGED=1
}

if [[ "$INSTALL_LIDAR_DEPS" == "1" ]]; then
  # Ensure packages required by run_flio_with_driver.launch are present.
  ensure_repo_at_commit \
    "$SlideSlamWs/src/ouster_example" \
    "https://github.com/ouster-lidar/ouster_example.git" \
    "43107a1"
  ensure_repo_at_commit \
    "$SlideSlamWs/src/ouster_decoder" \
    "https://github.com/KumarRobotics/ouster_decoder.git" \
    "d66b52d"
fi

if [[ "$DEPS_CHANGED" == "1" ]]; then
  # Dependency layout changed; clear stale per-package build cache.
  rm -rf \
    "$SlideSlamWs/build/ouster_ros" \
    "$SlideSlamWs/devel/.private/ouster_ros" \
    "$SlideSlamWs/logs/ouster_ros"
fi

# Allow RViz/X11 apps from container.
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

BUILD_CMD='set -e
cd /opt/slideslam_docker_ws
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
source /opt/slideslam_docker_ws/devel/setup.bash
echo "SlideSLAM workspace is built and sourced."'

KEEPALIVE_CMD='while true; do sleep 3600; done'

create_container() {
  docker run -d \
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
    bash -lc "$KEEPALIVE_CMD" >/dev/null
}

if docker ps -a --format "{{.Names}}" | awk -v target="$CONTAINER_NAME" '$0==target{found=1} END{exit(found?0:1)}'; then
  docker start "$CONTAINER_NAME" >/dev/null || true
else
  create_container
fi

# Old containers may have an entrypoint/cmd that exits immediately on start.
if ! docker ps --format "{{.Names}}" | awk -v target="$CONTAINER_NAME" '$0==target{found=1} END{exit(found?0:1)}'; then
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  create_container
fi

docker exec -it "$CONTAINER_NAME" bash -lc "$BUILD_CMD"
docker exec -it "$CONTAINER_NAME" bash -ic "source /opt/slideslam_docker_ws/devel/setup.bash; exec bash -i"