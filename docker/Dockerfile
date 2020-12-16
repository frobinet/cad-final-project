FROM ubuntu:20.04

RUN apt-get update -q && apt-get install -yqqq --no-install-recommends \
    gnupg2 curl ca-certificates wget python3-dev && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.2.89
ENV CUDA_PKG_VERSION 10-2=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update -q && apt-get install -yqqq --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-2 \
    && ln -s cuda-10.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.2 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441"
ENV NCCL_VERSION 2.7.8
RUN apt-get update -q && apt-get install -yqqq --no-install-recommends \
    cuda-nvml-dev-$CUDA_PKG_VERSION \
    cuda-command-line-tools-$CUDA_PKG_VERSION \
    cuda-nvprof-$CUDA_PKG_VERSION \
    cuda-npp-dev-$CUDA_PKG_VERSION \
    cuda-libraries-dev-$CUDA_PKG_VERSION \
    cuda-minimal-build-$CUDA_PKG_VERSION \
    libcublas-dev=10.2.2.89-1 \
    libnccl-dev=2.7.8-1+cuda10.2 \
    libnccl2=2.7.8-1+cuda10.2 \
    && apt-mark hold libnccl-dev \
    && rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs
ENV CUDNN_VERSION 7.6.5.32
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"
RUN apt-get update -q && apt-get install -yqqq --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.2 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.2 \
    && apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

RUN dpkg --add-architecture i386 && \
    apt-get update -q && apt-get install -yqqq --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

RUN apt-get update -q && apt-get install -yqqq --no-install-recommends \
        pkg-config \
    	libglvnd-dev libglvnd-dev:i386 \
	libgl1-mesa-dev libgl1-mesa-dev:i386 \
	libegl1-mesa-dev libegl1-mesa-dev:i386 \
	libgles2-mesa-dev libgles2-mesa-dev:i386 && \
    rm -rf /var/lib/apt/lists/*

# Install pip3 and libraries
RUN curl https://bootstrap.pypa.io/get-pip.py | python3

RUN pip3 install \
        wheel \
        torch==1.6.0 \
        matplotlib \ 
        jupyterlab \
        seaborn \
        tensorflow==2.3.0 \
        scikit-learn \
        pandas

# Add cuda libs to the path
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/cuda/lib64" >> ~/.bashrc

# TensorRT dependencies
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yqqq --no-install-recommends \
        libnvinfer6 \
        x11-apps \
        mesa-utils \ 
        git \
    && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists 

# Symlink CUDA 10.2 to 10.1 expected by PyTorch 
# We can just symlink to 10.2 version
RUN ln -s /usr/local/cuda/lib64/libcudart.so.10.2 /usr/local/cuda/lib64/libcudart.so.10.1
RUN ln -s /usr/bin/python3 /usr/bin/python

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

ENV CONTAINER_VERSION=0.1.0

# Jupyter lab widgets
RUN apt-get update -q && DEBIAN_FRONTEND=noninteractive apt-get install -yqqq \
        nodejs \
        npm \
    && \
    pip3 install ipywidgets && \
    jupyter nbextension enable --sys-prefix --py widgetsnbextension && \
    jupyter labextension install @jupyter-widgets/jupyterlab-manager@2.0 && \
    rm -rf /var/lib/apt/lists/* 

# Additional python dependencies
RUN pip3 install \
        attrdict==2.0.0 \
        gevent==1.4.0 \
        gevent-websocket==0.10.1 \
        greenlet==0.4.16 \
        h5py==2.6.0 \
        python-socketio==4.4.0 \
        ujson==1.35 \
        wsaccel==0.6.2 \
        Pillow

RUN sed -i 's/self.eio.send(sid, encoded_packet, binary=False)/self.eio.send(sid, encoded_packet)/g' /usr/local/lib/python3.8/dist-packages/socketio/server.py && \
    sed -i 's/self.eio.send(sid, ep, binary=binary)/self.eio.send(sid, ep)/g' /usr/local/lib/python3.8/dist-packages/socketio/server.py
