
FROM nvcr.io/nvidia/pytorch:21.12-py3

# Install ROS Noetic
RUN apt-get update -q && DEBIAN_FRONTEND=noninteractive apt-get install -yqqq lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yqqq \
        ros-noetic-desktop \
        ros-noetic-perception && \
    rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
ENV ROS_LANG_DISABLE="geneus;genlisp;gennodejs"

# Install python libs (tensorflow and pytorch already included)
RUN pip3 install \
        seaborn \
        attrdict==2.0.0 \
        gevent==1.4.0 \
        gevent-websocket==0.10.1 \
        greenlet==0.4.16 \
        python-socketio==4.4.0 \
        ujson==1.35 \
        wsaccel==0.6.2 \
        empy \
        catkin_pkg \
        rospkg

# NEEDED?
RUN sed -i 's/self.eio.send(sid, encoded_packet, binary=False)/self.eio.send(sid, encoded_packet)/g' /opt/conda/lib/python3.8/site-packages/socketio/server.py && \
    sed -i 's/self.eio.send(sid, ep, binary=binary)/self.eio.send(sid, ep)/g' /opt/conda/lib/python3.8/site-packages/socketio/server.py

# Create user
# UID and GID will be dynamically changed by VSCode to match the host user running the container
RUN addgroup --gid 1000 user && useradd user -u 1000 -g 1000 -m -s /bin/bash
USER user
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
