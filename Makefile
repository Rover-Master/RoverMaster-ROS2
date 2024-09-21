# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
SETUP_ENV:=source scripts/ros-env.sh
# Build time environment variables
# For debug build, use `CMAKE_ARGS=-DCMAKE_BUILD_TYPE=Debug make`
CMAKE_ARGS?=
# Ask CMake to generate compile_commands.json for each package
CMAKE_ARGS+=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
BUILD?=colcon build

all: build/deps
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS))
	$(info $(CMD))
	@ NONLOCAL=1 $(SETUP_ENV) && \
	  $(CMD); scripts/compile_commands.py

all/symlink: BUILD += --symlink-install
all/symlink: all

build/deps:
	@ mkdir -p build
	@ NONLOCAL=1 $(SETUP_ENV) && \
	  rosdep update && \
	  rosdep install -i --from-path src --rosdistro $${ROS_DISTRO} -y
	@ echo $$(date) > build/deps

PACKAGES:=$(shell scripts/package_name.py)
PACKAGES:=$(addprefix package/, $(PACKAGES))
$(PACKAGES): build/deps
	$(eval PACKAGE=$(shell basename $@))
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS) --packages-select $(PACKAGE))
	$(info $(CMD))
	@ NONLOCAL=1 BANNER="Building $(PACKAGE)" $(SETUP_ENV) && \
	  $(CMD); scripts/compile_commands.py

PACKAGES_LN:=$(addsuffix /symlink, $(PACKAGES))
$(PACKAGES_LN): BUILD += --symlink-install
$(PACKAGES_LN): build/deps
	$(eval PACKAGE=$(shell basename $(shell dirname $@)))
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS) --packages-select $(PACKAGE))
	$(info $(CMD))
	@ NONLOCAL=1 BANNER="Building $(PACKAGE)" $(SETUP_ENV) && \
	  $(CMD); scripts/compile_commands.py

# enumurate available launch files (for auto completion)
LAUNCH_FILES:=$(wildcard launch/*)
$(LAUNCH_FILES):
	@ BANNER="Launching $@" $(SETUP_ENV) && ros2 launch $@

sh shell bash:
	@ clear; \
	  ROS_DISTRO=$(ROS_DISTRO) \
	  bash --rcfile scripts/ros-env.sh || true

create:
	@ NONLOCAL=1 $(SETUP_ENV) && \
	  scripts/create.sh

clean:
	rm -rf build install .cache

.PHONY: build run clean $(LAUNCH_FILES) $(PACKAGES) sh shell bash
