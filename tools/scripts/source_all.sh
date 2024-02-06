#!/bin/bash

ROS_DISTRO_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_DISTRO_SOURCE}" ]; then
    source ${ROS_DISTRO_SOURCE}
fi

WS_LOCAL=install/setup.bash;
if [ -f "${WS_LOCAL}" ]; then
    source ${WS_LOCAL};
fi;