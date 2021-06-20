FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update -y && \
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

# # install yolov5-jp
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     rm -rf /yolov5_wsf && mkdir -p /yolov5_wsf/src && \
#     cd /yolov5_wsf/src && \
#     catkin_init_workspace && \
#     git clone https://github.com/Jpcaceres/yolov5.git && \
#     cd .. && \
#     catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
#     cd / && rm -rf /yolov5_wsf

# install libraries
RUN apt-get install -y ros-melodic-smach ros-melodic-smach-ros python-pip wget unzip\
    &&  PYTHONPATH=/usr/bin/python pip install future tqdm torch==1.4.0 torchvision==0.5.0 pathlib==1.0.1 scipy --no-cache-dir


# install VTK
RUN wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz && \
    tar -xf VTK-7.1.0.tar.gz && \
    cd VTK-7.1.0 && mkdir build && cd build && \
    cmake .. && \
    make -j12 && \
    make install && \
    cd ..

# install PCL    
RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.0.tar.gz && \
    tar -xf pcl-1.9.0.tar.gz && \
    cd pcl-pcl-1.9.0 && mkdir build && cd build && \
    cmake -DBUILD_visualization=ON .. && \
    make -j12 && \
    make install && \ 
    ldconfig

# install opencv 3.4
RUN wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/opencv_install.sh && \
    chmod +x opencv_install.sh && \
    ./opencv_install.sh

# install GPD
RUN git clone https://github.com/uchile-robotics-forks/gpd.git && \
    cd gpd && \
    mkdir build && cd build && \
    cmake .. && make -j10 && make install 

# create workspace folder
RUN mkdir -p /workspace/src

# copy our algorithm to workspace folder
ADD . /workspace/src

# install yolov5 package ros
RUN cd /workspace/src/ && git clone https://github.com/Jpcaceres/yolov5.git && cd yolov5 && cd src

RUN apt-get install ros-melodic-pcl-msgs

# install  package
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:usr/local/lib && cd /workspace/src/ && git clone https://github.com/uchile-robotics-forks/gpd_ros.git

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# compile and install our algorithm
RUN cd /workspace && /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO

# command to run the algorithm
CMD roslaunch robocup_challenge run.launch
