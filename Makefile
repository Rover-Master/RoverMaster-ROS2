# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
# Should have already been defined in setup scripts
ROS_DISTRO?=$(shell ls /opt/ros/ | tr ' ' '\n' | head -n 1)
# ROS2 Environment initialization
ROS_SETUP?=/opt/ros/$(ROS_DISTRO)/setup.bash
# Build time environment variables
BUILD_ENV?=
# Ask CMake to generate compile_commands.json for each package
BUILD_ENV+= CMAKE_EXPORT_COMPILE_COMMANDS=1

$(info Using ROS2/$(ROS_DISTRO))

COLCON_BUILD_COMMAND?=colcon build
build: build/deps
	@ echo $(COLCON_BUILD_COMMAND)
	@ source $(ROS_SETUP) && \
	  $(BUILD_ENV) $(COLCON_BUILD_COMMAND); \
	  scripts/compile_commands.py

build/deps:
	@ mkdir -p build
	@ rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@ echo $$(date) > build/deps

PACKAGES:=$(shell find . -iname package.xml | xargs scripts/package_name.py)
$(foreach p,$(PACKAGES),package/$(p)): build/deps
	$(eval PACKAGE=$(shell basename $@))
	@ echo
	@ echo "=================================================="
	@ echo "Building package $(PACKAGE)"
	@ echo "=================================================="
	@ source $(ROS_SETUP) && \
	  $(BUILD_ENV) $(COLCON_BUILD_COMMAND) --packages-select $(PACKAGE); \
	  scripts/compile_commands.py

# enumurate available launch files (for auto completion)
LAUNCH_FILES:=$(wildcard launch/*)
$(LAUNCH_FILES):
	@ echo
	@ echo "=================================================="
	@ echo "Launching $@"
	@ echo "=================================================="
	@ source install/setup.bash && ros2 launch $@

sh shell bash:
	@ clear; \
	  ROS_DISTRO=$(ROS_DISTRO) \
	  bash --rcfile scripts/ros-env.sh || true

clean:
	rm -rf build install .cache

.PHONY: build run clean $(LAUNCH_FILES) $(PACKAGES) sh shell bash
