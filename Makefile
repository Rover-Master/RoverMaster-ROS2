# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
# Should have already been defined in setup scripts
ROS_DISTRO?=iron
# ROS2 Environment initialization
ROS_SETUP?=/opt/ros/iron/setup.bash
# Build time environment variables
BUILD_ENV?=
# Ask CMake to generate compile_commands.json for each package
BUILD_ENV+= CMAKE_EXPORT_COMPILE_COMMANDS=1

COLCON_BUILD_COMMAND?=colcon build --symlink-install
build: build/deps
	@echo $(COLCON_BUILD_COMMAND)
	@source $(ROS_SETUP) && $(BUILD_ENV) $(COLCON_BUILD_COMMAND)
	@scripts/compile_commands.py

build/deps:
	@mkdir -p build
	@rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@echo $$(date) > build/deps

# enumurate available launch files (for auto completion)
LAUNCH_FILES:=$(wildcard launch/*)
$(LAUNCH_FILES): build
	@echo
	@echo "=================================================="
	@echo "Launching $@"
	@echo "=================================================="
	@source install/setup.bash && ros2 launch $@

install/setup.bash: build

sh shell bash: install/setup.bash
	tmux new -s $$(basename $(PWD)) "bash --rcfile scripts/bashrc.sh"

clean:
	rm -rf build install

.PHONY: build run clean $(LAUNCH_FILES) shell
