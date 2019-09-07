FROM ros:melodic-perception-bionic
#FROM pytorch/pytorch:1.1.0-cuda10.0-cudnn7.5-runtime

RUN apt-get update && apt-get install -y apt-transport-https wget && \
    rm -rf /var/lib/apt/lists/*

RUN echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub -O - | apt-key add -

COPY apt-get-requirements.txt /tmp/apt-get-requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/apt-get-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*

# Install ROS
ENV ROS_DISTRO melodic
RUN apt-get update && \
    apt-get install -y lsb-release && \
    rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y \
    python-catkin-tools \
    python-pip \
    ros-$ROS_DISTRO-sensor-msgs \
    && \
    rm -rf /var/lib/apt/lists/*


COPY pip-requirements.txt /tmp/pip-requirements.txt
RUN pip3 install -r /tmp/pip-requirements.txt

ENV WORKSPACE /home/root/code
RUN mkdir -p $WORKSPACE
RUN mkdir -p $WORKSPACE && \
    git clone https://github.com/wecacuee/votenet.git $WORKSPACE/votenet && \
    cd $WORKSPACE/votenet/pointnet2 && \
    python3 setup.py install

WORKDIR $WORKSPACE/votenet
CMD /opt/conda/bin/python $WORKSPACE/votenet/demo.py

ENV CATKIN_WS /home/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src && \
    cd $CATKIN_WS/src && \
    ln -sf $WORKSPACE/votenet/votenet_catkin votenet && \
    cd $CATKIN_WS && \
    catkin config \
      --init \
      --extend /opt/ros/$ROS_DISTRO  \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && \
    catkin build && \
    pip3 install git+https://github.com/eric-wieser/ros_numpy.git

ENV PYTHONPATH $WORKSPACE/votenet/:$PYTHONPATH
RUN echo " \
#!/bin/bash \
set -e \
 \
# setup ros environment \
source $CATKIN_WS/devel/setup.bash \
exec $@ \
" >   $CATKIN_WS/ros_entrypoint.sh \
 && chmod +x $CATKIN_WS/ros_entrypoint.sh
ENTRYPOINT $CATKIN_WS/ros_entrypoint.sh
CMD $CATKIN_WS/ros_entrypoint.sh rosrun votenet votenet_service.py


# nvidia-docker 1.0
ENV CUDA_VERSION 10.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"
