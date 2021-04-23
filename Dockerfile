# Copyright (c) 2019 TOYOTA MOTOR CORPORATION
# Copyright (c) 2020 MID Academic Promotions, Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

ARG BASE_IMAGE

FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

## Added these requirements to the repository.debs in mbot_simulation_sa so it is faster
RUN sudo apt-get install ros-$ROS_DISTRO-serial -y

RUN sudo apt-get install python-catkin-tools -y

RUN sudo apt-get install ros-$ROS_DISTRO-rqt -y

RUN mkdir /robocup_ws
ADD src /robocup_ws/src
RUN cd /robocup_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace || true
RUN cd /robocup_ws/src/mbot_simulation_sa/ && ./repository.debs

#RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /robocup_ws/src/tmc_wrs_gazebo/mbot_simulation_sa/resources/packages/monarch_msgs && catkin build --this

#RUN cd /robocup_ws && source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN cd /robocup_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin config --install-space /opt/ros/$ROS_DISTRO && catkin config --install && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0
#RUN cd /robocup_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0

#ADD filterable-rosmaster.py /opt/ros/melodic/bin/
#RUN rm /opt/ros/$ROS_DISTRO/bin/rosmaster && ln -s /opt/ros/$ROS_DISTRO/bin/filterable-rosmaster.py /opt/ros/$ROS_DISTRO/bin/rosmaster

RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosrun tmc_gazebo_task_evaluators setup_score_widget
#&& rosrun tmc_gazebo_task_evaluators setup_score_widget
# && source /robocup_ws/devel/setup.bash

ADD supervisord.conf /etc/supervisor/supervisord.conf

VOLUME [ \
    "/opt/ros/melodic/share/hsrb_description", \
    "/opt/ros/melodic/share/hsrb_meshes", \
    "/opt/ros/melodic/share/tmc_wrs_gazebo_worlds", \
    "/opt/ros/melodic/share/gazebo_ros", \
    "/opt/ros/melodic/lib/gazebo_ros", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/gazebo_ros", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/gazebo_msgs", \
    "/opt/ros/melodic/share/hsrb_rosnav_config", \
    "/opt/ros/melodic/share/tmc_control_msgs", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/tmc_control_msgs", \
    "/opt/ros/melodic/include/tmc_control_msgs" \
    ]

CMD ["/usr/local/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]

