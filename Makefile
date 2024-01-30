# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
# Should have already been defined in setup scripts
ROS_DISTRO?=iron

build: build/deps
	CMAKE_EXPORT_COMPILE_COMMANDS=1 colcon build --symlink-install
	@scripts/compile_commands.py

launch/%.xml: build
	@echo
	@echo "=================================================="
	@echo "Launching $@"
	@echo "=================================================="
	@source install/setup.bash && ros2 launch $@

shell: build
	source install/setup.bash && bash

build/deps:
	@mkdir -p build
	@rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@echo $$(date) > build/deps

clean:
	rm -rf build install

.PHONY: build run clean launch/%.xml shell
 