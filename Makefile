# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
SETUP_ENV:=source scripts/ros-env.sh
# Build time environment variables
BUILD_ENV?=
# Ask CMake to generate compile_commands.json for each package
BUILD_ENV+= CMAKE_EXPORT_COMPILE_COMMANDS=1

BUILD?=colcon build
build: build/deps
	@ NONLOCAL=1 $(SETUP_ENV) && \
	  $(BUILD_ENV) $(BUILD); \
	  scripts/compile_commands.py

build/deps:
	@ mkdir -p build
	@ rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@ echo $$(date) > build/deps

PACKAGES:=$(shell find src -iname package.xml | xargs scripts/package_name.py)
$(addprefix package/, $(PACKAGES)): build/deps
	$(eval PACKAGE=$(shell basename $@))
	@ NONLOCAL=1 BANNER="Building $(PACKAGE)" $(SETUP_ENV) && \
	  $(BUILD_ENV) $(BUILD) --packages-select $(PACKAGE); \
	  scripts/compile_commands.py

# enumurate available launch files (for auto completion)
LAUNCH_FILES:=$(wildcard launch/*)
$(LAUNCH_FILES):
	@ BANNER="Launching $@" $(SETUP_ENV) && ros2 launch $@

sh shell bash:
	@ clear; \
	  ROS_DISTRO=$(ROS_DISTRO) \
	  bash --rcfile scripts/ros-env.sh || true

clean:
	rm -rf build install .cache

.PHONY: build run clean $(LAUNCH_FILES) $(PACKAGES) sh shell bash
