# In order to support graphical interfaces,
# this should be run with 
# docker run -it --rm \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \  
#     ros-indigo-full-catkin <cmd> 
#
# The -e and -v commands are needed to display on the host X server.
# For hardware support, you will also need:
#  --privileged   (to access the graphics card) 
#  It may also be required to call
#  $ xhost +
#  before running the container.

FROM jenniferbuehler/ros-indigo-full-catkin 

MAINTAINER Jennifer Buehler

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
#    ros-indigo-shape-tools \
    ros-indigo-laser-filters \
    ros-indigo-openni-launch \
    && rm -rf /var/lib/apt/lists/

COPY common_sensors /catkin_ws/src/common_sensors

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
