#!/bin/bash
# ============================================================
# This script is used to setup interactive shell environment.
# It is intended for Make scripts, do not run directly.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DIVIDER="============================================================"
echo -e "\e[90m${DIVIDER}\e[0m"
# Display banner (optional)
if [ ! -z "${BANNER}" ]; then
    echo -e "\e[34m>>>>>> \e[0;90m \e[0;1;36m${BANNER}\e[0;90m"
    unset BANNER
fi
# Source user environment file, if exists
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
elif [ -f ~/.bash_profile ]; then
    source ~/.bash_profile
elif [ -f ~/.profile ]; then
    source ~/.profile
fi
# Check for ROS2 installation
source scripts/ros-env.sh
# Command to find PWD
CMD_PWD='$('"pwd | sed 's@^${ROS_WS}\?@.@'"')'
# Rewrite the PS1 prompt to highlight current ROS2 environment
export PS1="\033[034mROS2::${ROS_DISTRO}\[\033[00m\]"
export PS1="${PS1} \[\033[04;32m\]$(basename $ROS_WS)\[\033[00m\]"
export PS1="${PS1} \[\033[96m\]${CMD_PWD}\[\033[00m\] \$ "
# Function to launch ROS2 manifests in ./launch/
function launch() {
    LAUNCH=${ROS_WS}/launch
    if [ -f ${LAUNCH}/$1 ]; then
        ros2 launch ${LAUNCH}/$1
    elif [ -f ${LAUNCH}/$1.py ]; then
        ros2 launch ${LAUNCH}/$1.py
    elif [ -f ${LAUNCH}/$1.xml ]; then
        ros2 launch ${LAUNCH}/$1.xml
    elif [ -f ${LAUNCH}/$1.yml ]; then
        ros2 launch ${LAUNCH}/$1.yml
    elif [ -f ${LAUNCH}/$1.yaml ]; then
        ros2 launch ${LAUNCH}/$1.yaml
    else
        echo "========================"
        if [ -z "$1" ]; then
            echo "Usage: launch <file>"
            echo "========================"
        else
            echo "File not found: $1"
            echo "========================"
        fi
        echo "Available launch files:"
        for file in launch/*; do
            echo "- $(basename $file)"
        done
        echo
    fi
}
# Add completion for launch function
LAUNCH_FILES=
for file in launch/*; do
    LAUNCH_FILES+="$(basename $file) "
done
complete -W "$LAUNCH_FILES" launch
# Source additional environment files if exists
for file in *.sh; do
    if [ -f "$file" ]; then
        echo -e "\e[33m[INFO] \e[0;90m Sourcing additional env: \e[4m$file\e[0m"
        source $file
    fi
done
# Include runtime bin path
export PATH="$PATH:$SCRIPT_DIR/runtime-bin"
echo -e "\e[90m${DIVIDER}\e[0m"
