#!/bin/bash
ARGS=("ros2" "pkg" "create")
LICENSE="MIT"
PACKAGE_NAME=""
cd src
echo "Workspace source directory: $(pwd)"

# Ask for build type
read -p "Build system [ament_cmake]: " BUILD_TYPE
BUILD_TYPE=${BUILD_TYPE:-ament_cmake}
ARGS+=("--build-type" "$BUILD_TYPE")

# Ask for license type
read -p "License type [MIT]: " LICENSE
LICENSE=${LICENSE:-MIT}
ARGS+=("--license" "$LICENSE")

# Ask for package name
while [ "$PACKAGE_NAME" = "" ]; do
    if read -p "Package name: " PACKAGE_NAME; then
        if [ -e "$PACKAGE_NAME" ]; then
            echo "Package $PACKAGE_NAME already exists!"
            PACKAGE_NAME=""
        fi
    else # User pressed CTRL-D
        echo; echo "Aborted ..."
        exit 1
    fi
done

# Ask for node names
while true; do
    if read -p "Node name (enter=skip): " NODE_NAME; then
        if [ "$NODE_NAME" = "" ]; then
            break
        else
            ARGS+=("--node-name" "$NODE_NAME")
        fi
    else # User pressed CTRL-D
        echo; break
    fi
done

ARGS+=("$PACKAGE_NAME")

echo ${ARGS[*]}; ${ARGS[*]}
