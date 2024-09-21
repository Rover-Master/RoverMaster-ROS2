#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $(dirname $SCRIPT_DIR)
echo "Starting debug session at $PWD..."
source scripts/ros-env.sh

echo PROGRAM=$PROGRAM

PROGRAM=install/orb_slam3/lib/orb_slam3/monocular
ARGS="--ros-args --ros-args -r __ns:=/slam"

# Write temporary param file
echo "\
/**:
  ros__parameters:
    voc_file: assets/ORBvoc.txt
    settings_file: assets/EuRoC.yaml
    use_imu: true
" > /tmp/launch_params_orb_slam3

ARGS="${ARGS} --params-file /tmp/launch_params_orb_slam3"

echo "${PROGRAM} ${ARGS}"

/usr/bin/gdb \
  --ex "set environment LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
  $@ \
  --args ${PROGRAM} ${ARGS} \
;
