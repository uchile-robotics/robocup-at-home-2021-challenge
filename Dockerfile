FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && \
    apt-get install -y git ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy ros-$ROS_DISTRO-geometry && \
    apt-get clean

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

# install hsrb_moveit_config
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /hsrb_moveit_config/src && \
    cd /hsrb_moveit_config/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/pipperv/hsrb_moveit_config.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /hsrb_moveit_config

# install yolov5-jp
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /yolov5/src && \
    cd /yolov5/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/Jpcaceres/yolov5.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /yolov5

# install libraries
RUN apt-get install -y ros-melodic-smach ros-melodic-smach-ros python-pip \
    &&  PYTHONPATH=/usr/bin/python pip install future tqdm torch==1.4.0 torchvision==0.5.0 pathlib==1.0.1 scipy --no-cache-dir

# cambiar activacion
#RUN cd /home/developer/ 

# create workspace folder
RUN mkdir -p /workspace/src

# copy our algorithm to workspace folder
ADD . /workspace/src

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# compile and install our algorithm
RUN cd /workspace && /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO

# command to run the algorithm
CMD roslaunch robocup_challenge run.launch
