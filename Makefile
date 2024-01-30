# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
# Should have already been defined in setup scripts
ROS_DISTRO?=iron

build: build/deps
	colcon build --symlink-install
	@scripts/compile_commands.py

run: build nodes.list
	bash scripts/run.sh

build/deps:
	@mkdir -p build
	@rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@echo $$(date) > build/deps

clean:
	rm -rf build install

.PHONY: build run clean
