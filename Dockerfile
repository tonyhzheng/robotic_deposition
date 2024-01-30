FROM osrf/ros:melodic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,display

RUN apt-get update && apt-get install -y \
  software-properties-common \
  lsb-release ffmpeg\
  wget dbus pluma apt-utils curl sudo vim \
  ros-melodic-desktop-full \
  mesa-utils iputils-ping\
  libgmp3-dev libosmesa6 libgl1-mesa-glx libglfw3 libosmesa6-dev libccd-dev libglew-dev libsdl-dev libxslt-dev\
  unzip libgl1-mesa-dev xserver-xorg-dev redis-server\ 
  apt-transport-https x11-apps iproute2  net-tools usbutils v4l-utils\
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
###############################################################################

#apt pkg not found
RUN cd /usr/lib/python3/dist-packages && ln -s apt_pkg.cpython-36m-x86_64-linux-gnu.so apt_pkg.so
RUN sudo ln -s /usr/lib/python3/dist-packages/gi/_gi.cpython-36m-x86_64-linux-gnu.so /usr/lib/python3/dist-packages/gi/_gi.cpython-37m-x86_64-linux-gnu.so

RUN curl -o /usr/local/bin/patchelf https://s3-us-west-2.amazonaws.com/openai-sci-artifacts/manual-builds/patchelf_0.9_amd64.elf \
    && chmod +x /usr/local/bin/patchelf

RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -a -G plugdev docker
RUN usermod -a -G video docker

USER docker
RUN mkdir -p /home/docker
RUN touch ~/.sudo_as_admin_successful
USER root

WORKDIR /home/docker

# Python package management and basic dependencies
RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get install -y python3.7 python3.7-dev python3.7-distutils g++-7  python3-dev python3.7-tk python-tk python3-gi-cairo libcanberra-gtk3-module
RUN apt-get purge python3-numpy -y

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 
RUN update-alternatives --config gcc

# Register the version in alternatives
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1

# Upgrade pip to latest version
RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py --force-reinstall && \
    rm get-pip.py

ENV PATH $PATH:/home/docker/.local/bin
USER docker
COPY requirements.txt /home/docker/requirements.txt
RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install -r /home/docker/requirements.txt


USER root
RUN pip3 install cmake --upgrade

RUN cd /usr/lib/x86_64-linux-gnu && sudo ln -sf libboost_python3.so libboost_python.so &&  sudo ln -sf libboost_python3.a libboost_python.a
RUN cd /usr/lib/x86_64-linux-gnu && sudo ln -sf libboost_numpy3.so libboost_numpy.so &&  sudo ln -sf libboost_numpy3.a libboost_numpy.a

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&  \
                  echo -e 'export HOME=/home/docker \n' >> /root/.bashrc && \
                  echo -e 'source /home/docker/bind_mount/devel/setup.bash \n' >> /root/.bashrc && \
                  echo -e 'source /opt/ros/melodic/setup.bash'  >> /root/.bashrc"


USER docker
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&  \
                  echo -e 'export HOME=/home/docker \n' >> ~/.bashrc && \
                  echo -e 'source /home/docker/bind_mount/devel/setup.bash \n' >> ~/.bashrc && \
                  echo -e 'source /opt/ros/melodic/setup.bash'  >> ~/.bashrc"

