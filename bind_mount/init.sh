#!/bin/bash
set -e
rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
# setup ros environment
# source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
