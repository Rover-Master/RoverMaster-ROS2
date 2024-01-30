#!/bin/bash
source /opt/ros/iron/setup.bash
source ./install/local_setup.bash

ARGS=()
SPACER="new-session"
FLAG=""

while read -r line; do
    ARGS+=($SPACER $FLAG 'scripts/run_node.sh' $line ';')
    if ["$FLAG" = "-h"]; then
        FLAG="-v"
    else
        FLAG="-h"
    fi;
    SPACER="split-window"
done <<<$(cat nodes.list)

echo tmux ${ARGS[*]}
tmux ${ARGS[*]}
