# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
# Should have already been defined in setup scripts
ROS_DISTRO?=iron
# ROS2 Environment initialization
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
ROS_SETUP?=/opt/ros/iron/setup.bash
ENV?=
# Ask CMake to generate compile_commands.json for each package
ENV+= CMAKE_EXPORT_COMPILE_COMMANDS=1

COLCON_BUILD_COMMAND?=colcon build --symlink-install
build: build/deps
	@echo $(COLCON_BUILD_COMMAND)
	@source $(ROS_SETUP) && $(ENV) $(COLCON_BUILD_COMMAND)
	@scripts/compile_commands.py

build/deps:
	@mkdir -p build
	@rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@echo $$(date) > build/deps

launch/%.xml: build
	@echo
	@echo "=================================================="
	@echo "Launching $@"
	@echo "=================================================="
	@source install/setup.bash && ros2 launch $@

install/setup.bash: build

shell: install/setup.bash
	tmux new -s $$(basename $(PWD)) "bash --rcfile scripts/bashrc.sh"

clean:
	rm -rf build install

.PHONY: build run clean launch/%.xml shell
