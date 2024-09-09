# ============================================================
# This script is used to setup ROS2 environment for the current shell
# It will source the ROS2 setup.bash file and set bash prompt accordingly.
# If local environment is not found, it will fallback to global environment.
# It is intended to be used by ../Makefile, do not run directly.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
echo -e "\e[90m============================================================\e[0m"
# Source user environment file, if exists
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
elif [ -f ~/.bash_profile ]; then
    source ~/.bash_profile
elif [ -f ~/.profile ]; then
    source ~/.profile
fi
# Check for ROS2 installation
if [ -z "$ROS_DISTRO" ]; then
    ROS_DISTRO=$(ls /opt/ros/ | tr ' ' '\n' | tail -n 1)
fi
if [ -z "$ROS_DISTRO" ]; then
    echo -e  "\e[31m[ERROR]\e[0;90m ROS2 installation not found on this system"
    echo -e  "\e[33m[INFO]\e[0;90m Searched: /opt/ros/"
    read -n 1 -s -r -p "Press any key to exit..."
    exit 1;
fi
# Source ROS2 setup.bash file
if [ -f install/local_setup.bash ]; then
    source install/setup.bash
    echo -e "\e[33m[INFO]\e[0;90m Using global ROS2 environment: $ROS_DISTRO"
    WS=$PWD
else
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo -e "\e[33m[WARN]\e[0;90m Local environment not found."
    echo -e  "\e[32m[INFO]\e[0;90m Using global ROS2 environment: \e[4m/opt/ros/$ROS_DISTRO\e[0m"
    WS=/opt/ros/$ROS_DISTRO
fi
echo -e "\e[90m============================================================\e[0m"
# Command to find PWD
CMD_PWD='$('"pwd | sed 's@^${WS}\?@.@'"')'
# Rewrite the PS1 prompt to highlight current ROS2 environment
export PS1="\033[034mROS2::${ROS_DISTRO}\[\033[00m\]"
export PS1="${PS1} \[\033[04;32m\]$(basename $WS)\[\033[00m\]"
export PS1="${PS1} \[\033[96m\]${CMD_PWD}\[\033[00m\] \$ "
# Function to launch ROS2 manifests in ./launch/
function launch() {
    if [ -f launch/$1 ]; then
        ros2 launch launch/$1
    elif [ -f launch/$1.py ]; then
        ros2 launch launch/$1.py
    elif [ -f launch/$1.xml ]; then
        ros2 launch launch/$1.xml
    elif [ -f launch/$1.yml ]; then
        ros2 launch launch/$1.yml
    elif [ -f launch/$1.yaml ]; then
        ros2 launch launch/$1.yaml
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