#!/usr/bin/env bash
# The Replanner under ROS without a simulator (docs/LOCAL_CHECKLIST.md L-R, L-B1, L-E):
# a scratch build of blueboat_replanner, then test/ros_graph_check.py on an ISOLATED ROS domain.
#
#   .claude/tools/ros_check.sh [scratch directory] [arguments of test.ros_graph_check]
#   default scratch directory: ${TMPDIR:-/tmp}/replanner_ros_iso      (about 15 MB)
#   example: .claude/tools/ros_check.sh /tmp/replanner_ros_iso --c4-speedup 1
#
# - Real: BlueBoat-Belief's installed node, this package's two nodes and launch files, DDS,
#   every message and service type. Made up by the check: sonar pings, odometry, GPS, a world
#   file, and Control (progress + the /blueboat/update_path service). No Gazebo, no boat.
# - Nothing is written under ~/ros2_ws/{build,install,log}: the build, the install and the
#   colcon log go to the scratch directory. The underlay (blueboat_interfaces,
#   blueboat_belief_interfaces, blueboat_belief) is read from ~/ros2_ws/install.
# - It refuses to start while a simulation recording is running on this machine, and the check
#   itself refuses ROS domain 0 and a graph that is not empty.
# - About 28 minutes: the C4 mission of [R7] runs at real speed (see --c4-speedup).
# Exit codes: the check's (0 pass, 1 a check failed, 3 not isolated); 2 a recording is running;
# 4 the build failed.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WS="${ROS2_WS:-$HOME/ros2_ws}"
ISO="${1:-${TMPDIR:-/tmp}/replanner_ros_iso}"
[ $# -gt 0 ] && shift
DOMAIN="${REPLANNER_CHECK_DOMAIN:-47}"

if pgrep -f 'record_ru[n]\.py' > /dev/null; then
  echo "a simulation recording is running: not started"; exit 2
fi
if [ "$DOMAIN" = "0" ]; then
  echo "REPLANNER_CHECK_DOMAIN must not be 0"; exit 3
fi
mkdir -p "$ISO" || exit 4
STAMP="$(date +%Y%m%d-%H%M%S)"

# a clean environment: nothing of the calling shell (a sourced workspace, a venv) leaks in
env -i HOME="$HOME" LANG=C.UTF-8 PATH=/usr/bin:/bin PYTHONDONTWRITEBYTECODE=1 \
    bash --noprofile --norc -c "
  source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' || exit 4
  cd '$ISO' || exit 4
  nice -n 19 colcon --log-base '$ISO/log' build --base-paths '$ROOT' \
      --packages-select blueboat_replanner --build-base '$ISO/build' \
      --install-base '$ISO/install' --executor sequential || exit 4
  source '$ISO/install/setup.bash' || exit 4
  export ROS_DOMAIN_ID=$DOMAIN FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_PYTHON_CHECK_FIELDS=1
  export ROS_LOG_DIR='$ISO/roslog'
  cd '$ROOT/blueboat_replanner' || exit 4
  exec /usr/bin/python3 -B -m test.ros_graph_check --work '$ISO/work/$STAMP' $*"
