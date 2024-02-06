.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/

.PHONY: clean-docs
clean-docs:
	@rm -rf generated-docs/

.PHONY: purge
purge:
	@rm -rf build/ install/ log/ logs/ generated-docs/

.PHONY: clean-test
clean-test:
	source ./tools/scripts/source_all.sh
	colcon test-result --delete-yes

.PHONY: build-debug
build-debug:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to ${PACKAGES}
	fi

.PHONY: docs
docs:
	doxygen Doxyfile

.PHONY: build
build:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=OFF
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=OFF --packages-up-to ${PACKAGES}
	fi

.PHONY: build-debug-lockfree
build-debug-lockfree:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=ON
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=ON --packages-up-to ${PACKAGES}
	fi

.PHONY: build-lockfree
build-lockfree:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=ON
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DENABLE_LOCKFREE=ON --packages-up-to ${PACKAGES}
	fi

.PHONY: test
test:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon test --return-code-on-test-failure; colcon test-result --verbose
	else
		colcon test --return-code-on-test-failure --packages-up-to ${PACKAGES}; colcon test-result --verbose
	fi

.PHONY: test-cpp
test-cpp:
	source ./tools/scripts/source_all.sh
	ament_clang_format --config .clang-format ${PATHS}
	ament_clang_tidy --config .clang-tidy build/compile_commands.json

.PHONY: reformat
reformat:
	source ./tools/scripts/source_all.sh
	autoflake --in-place --remove-unused-variables --remove-all-unused-imports --ignore-init-module-imports -r ${PATHS}
	black -l 99 ${PATHS}
	ament_clang_format --config .clang-format --reformat ${PATHS}

.PHONY: rosdep-install
rosdep-install:
	source ./tools/scripts/source_all.sh
	sudo apt update
	rosdep update
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src

.PHONY: rosdep-install-list
rosdep-install-list:
	source ./tools/scripts/source_all.sh
	rosdep update --include-eol-distros
	rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -r -y --rosdistro ${ROS_DISTRO} --from-paths . | sort >> tools/image/ros-deps

.PHONY: build-docker-cpu-iron
build-docker-cpu-iron:
	@IMG_NAME=${IMG_NAME}
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target final_image \
		--build-arg BASE_IMAGE=osrf/ros:iron-desktop \
		-t ${IMG_NAME} .

.PHONY: get-ownership
get-ownership:
	sudo chown -R ${USER} *

.PHONY: join-session
join-session:
	@CONT_NAME="${CONT_NAME}"
	docker exec -it ${CONT_NAME} /bin/bash
