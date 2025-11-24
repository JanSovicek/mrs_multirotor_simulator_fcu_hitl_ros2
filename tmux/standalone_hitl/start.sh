#!/bin/bash

# Set the workspace root
export ROS2_WS=~/ros2_workspace

# Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source the workspace overlay
source $ROS2_WS/install/setup.bash

export AMENT_PREFIX_PATH=$ROS2_WS/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$ROS2_WS/install/mrs_multirotor_simulator/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$ROS2_WS/src:$ROS_PACKAGE_PATH

# Make sure plugin libraries are found
export LD_LIBRARY_PATH=$ROS2_WS/install/mrs_multirotor_simulator/lib:$LD_LIBRARY_PATH

# Make sure plugin XML files are found
export PLUGINLIB_DIR=$ROS2_WS/install/mrs_multirotor_simulator/share/mrs_multirotor_simulator

# Optional: ensure ROS_PACKAGE_PATH includes src
export ROS_PACKAGE_PATH=$ROS2_WS/src:$ROS_PACKAGE_PATH

export AMENT_PREFIX_PATH=$ROS2_WS/install:$AMENT_PREFIX_PATH

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

# start tmuxinator
tmuxinator start -p ./session.yml

# if we are not in tmux
if [ -z $TMUX ]; then

  # just attach to the session
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME

# if we are in tmux
else

  # switch to the newly-started session
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME" 

fi