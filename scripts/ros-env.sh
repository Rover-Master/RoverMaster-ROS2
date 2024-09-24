#!/bin/bash
# ============================================================
# Auto detection and activation of ROS2 environment
# If local setup script is not found, fallback to global setup.
# It is intended for Make scripts, do not run directly.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
# Display banner (optional)
if [ ! -z "${BANNER}" ]; then
    echo -e "\e[34m>>>>>> \e[0;90m \e[0;1;36m${BANNER}\e[0;90m"
    unset BANNER
fi
if [ -z "$ROS_DISTRO" ]; then
    export ROS_DISTRO=$(ls /opt/ros/ | tr ' ' '\n' | tail -n 1)
fi
if [ -z "$ROS_DISTRO" ]; then
    echo -e "\e[31m[ERROR]\e[0;90m ROS2 installation not found on this system"
    echo -e "\e[32m[INFO] \e[0;90m Searched: /opt/ros/"
    read -n 1 -s -r -p "\e[33mPress any key to exit...\e[0m"
    exit 1;
else
    echo -e "\e[32m[INFO] \e[0;90m ROS2 Distribution: \e[4m${ROS_DISTRO}\e[0m"
fi
# Source ROS2 setup.bash file
if [ -f "install/local_setup.bash" ] && [ -z "${NONLOCAL}" ]; then
    source install/setup.bash
    echo -e "\e[32m[INFO] \e[0;90m Using project local ROS2 environment"
    export ROS_WS=$PWD
else
    source /opt/ros/$ROS_DISTRO/setup.bash
    if [ -z "${NONLOCAL}" ]; then
        echo -e "\e[33m[WARN] \e[0;90m Local environment not found."
    fi
    echo -e "\e[32m[INFO] \e[0;90m Using global ROS2 environment: \e[4m/opt/ros/$ROS_DISTRO\e[0m"
    export ROS_WS=/opt/ros/$ROS_DISTRO
fi
