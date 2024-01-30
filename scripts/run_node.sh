#!/bin/bash
ros2 run $@

echo "

============================================================
Node [$@] exited with code $?"

read -n 1 -s -r -p "Press any key to continue ..."
