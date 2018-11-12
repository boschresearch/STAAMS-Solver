# Copyright (c) 2018 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#
# @author: Jan Behrens

FROM ros:kinetic-ros-core

# install graph-tool
RUN echo "deb http://downloads.skewed.de/apt/xenial xenial universe" >> /etc/apt/sources.list
RUN echo "deb-src http://downloads.skewed.de/apt/xenial xenial universe" >> /etc/apt/sources.list
RUN apt-key adv --keyserver pgp.skewed.de --recv-key 612DEFB798507F25

# RUN cat /etc/apt/sources.list

RUN apt-get update && apt-get install -y --allow-unauthenticated \
    build-essential python-pip python-graph-tool liblmdb0 lmdb-utils apt-utils

RUN pip install --upgrade lmdb pip catkin_tools ortools

# Create ROS workspace
COPY . /ws/src/staams-solver/
WORKDIR /ws

# Install the package dependencies
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

# Set up the development environment
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && \
    catkin config --install && \
    catkin build && catkin run_tests"
