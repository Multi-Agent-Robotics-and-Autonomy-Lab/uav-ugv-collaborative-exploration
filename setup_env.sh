#!/bin/bash
# Source this in EVERY terminal before launching anything:
#     source /path/to/uav-ugv-collaborative-exploration/setup_env.sh
#
# Two footguns this protects against:
#  1. If another catkin workspace's gazeboSetup.bash is sourced from ~/.bashrc,
#     it can pin GAZEBO_PLUGIN_PATH at that workspace's OLD quadcopter plugin
#     (one without the NaN / load_factor guards). Rebuilding this workspace then
#     has NO effect and the drone teleports to the origin. We PREPEND here so
#     this workspace always wins.
#  2. conda's python has no rospkg, so xacro dies with
#     "substitution args not supported: No module named 'rospkg'".

# Resolve the workspace root from this script's own location, so the repo works
# from any clone path.
WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
UAVSIM=$WS/src/CERLAB-UAV-Autonomy/uav_simulator

while [ -n "$CONDA_DEFAULT_ENV" ] && command -v conda >/dev/null 2>&1; do
    conda deactivate 2>/dev/null || break
done
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "[env] WARNING: still in conda env '$CONDA_DEFAULT_ENV' - run 'conda deactivate'"
fi

source /opt/ros/noetic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash"
else
    echo "[env] WARNING: $WS/devel/setup.bash not found - run 'catkin_make' first"
fi
source "$UAVSIM/gazeboSetup.bash"

# This workspace FIRST so it beats any other on the path.
export GAZEBO_PLUGIN_PATH=$UAVSIM/plugins:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$UAVSIM/models:$GAZEBO_MODEL_PATH

echo "[env] workspace : $(rospack find uav_simulator 2>/dev/null)"
echo "[env] plugin dir: $(echo $GAZEBO_PLUGIN_PATH | cut -d: -f1)"
echo "[env] python    : $(command -v python3)"
