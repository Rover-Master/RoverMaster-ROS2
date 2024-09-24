# Change default shell from `sh` to `bash` so we have `source` command available
SHELL:=/bin/bash
CLR_ENV:=source scripts/clear-env.sh
ROS_ENV:=source scripts/ros-env.sh
# Find system python3 for CMake
PYTHON3:=$(shell env -i which python3)
# Build time environment variables
# For debug build, use `CMAKE_ARGS=-DCMAKE_BUILD_TYPE=Debug make`
CMAKE_ARGS?=
# Ask CMake to generate compile_commands.json for each package
CMAKE_ARGS+=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
CMAKE_ARGS+=-DPython3_EXECUTABLE=$(PYTHON3)
BUILD?=colcon build

all: build/deps
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS))
	@ $(CLR_ENV) && \
	  NONLOCAL=1 BANNER="$(CMD)" $(ROS_ENV) && \
	  $(CMD); scripts/compiledb.py

all/symlink: BUILD += --symlink-install
all/symlink: all

build/deps:
	@ mkdir -p build
	@ $(CLR_ENV) && \
	  NONLOCAL=1 $(ROS_ENV) && \
	  rosdep update && \
	  rosdep install -i \
			--from-path src \
			--rosdistro $${ROS_DISTRO} \
			-y
	@ echo $$(date) > build/deps

PACKAGES:=$(shell scripts/package_name.py)
PACKAGES:=$(addprefix package/, $(PACKAGES))
$(PACKAGES): build/deps
	$(eval PACKAGE=$(shell basename $@))
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS) --packages-select $(PACKAGE))
	$(info $(CMD))
	@ $(CLR_ENV) \
	  && NONLOCAL=1 BANNER="Building $(PACKAGE)" $(ROS_ENV) && \
	  $(CMD); scripts/compile_commands.py

PACKAGES_LN:=$(addsuffix /symlink, $(PACKAGES))
$(PACKAGES_LN): BUILD += --symlink-install
$(PACKAGES_LN): build/deps
	$(eval PACKAGE=$(shell basename $(shell dirname $@)))
	$(eval CMD=$(BUILD) --cmake-args $(CMAKE_ARGS) --packages-select $(PACKAGE))
	$(info $(CMD))
	@ $(CLR_ENV) && \
	  NONLOCAL=1 BANNER="Building $(PACKAGE)" $(ROS_ENV) && \
	  $(CMD); scripts/compile_commands.py

# enumurate available launch files (for auto completion)
LAUNCH_FILES:=$(wildcard launch/*)
$(LAUNCH_FILES):
	@ $(CLR_ENV) && \
	  BANNER="Launching $@" $(ROS_ENV) && \
		ros2 launch $@

sh shell bash:
	@ clear && bash --rcfile scripts/shell.sh || true

create:
	@ $(CLR_ENV) && \
	  NONLOCAL=1 $(ROS_ENV) && \
	  scripts/create.sh

clean:
	rm -rf build install .cache

.PHONY: build run clean $(LAUNCH_FILES) $(PACKAGES) sh shell bash
