ARG BASE_IMAGE

FROM $BASE_IMAGE as base
LABEL maintainer="Siddharth Saha <sisahawork@gmail.com>"
SHELL ["/bin/bash", "-o", "pipefail", "-ic"]

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  ca-certificates \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  software-properties-common \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  build-essential \
  curl \
  rsync \
  wget \
  sudo \
  vim \
  tmux \
  tmuxp \
  git \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-pip \
  python3-argcomplete \
  bash-completion \
  clang-tidy \
  cmake \
  ccache \
  lld \
  htop \
  doxygen \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed numpy autoflake black mcap mcap-ros2-support pandas seaborn matplotlib tqdm pyperclip

FROM base as ros_deps_generator

COPY src /docker_bld_tmp/src

RUN cd /docker_bld_tmp && \
    apt update && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive rosdep install --as-root "apt:false pip:false" --simulate --reinstall -r -y --ignore-src --rosdistro ${ROS_DISTRO} --from-paths . | sort >> /ros-deps && \
    apt clean && \
    rm -rf /docker_bld_tmp /var/lib/apt/lists/*

FROM base as ros_deps

COPY --from=ros_deps_generator /ros-deps /ros-deps

RUN apt update && \
  rosdep update && \
  DEBIAN_FRONTEND=noninteractive /bin/bash /ros-deps && \
  apt clean && \
  rm -rf /var/lib/apt/lists/* /ros-deps

FROM ros_deps as final_image

COPY tools/image/cyclonedds.xml /etc/cyclone/cyclonedds.xml

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=\"rmw_cyclonedds_cpp\"" >> /root/.bashrc && \
    echo "export CYCLONEDDS_URI=\"file:///etc/cyclone/cyclonedds.xml\"" >> /root/.bashrc && \
    echo "export CYCLONE_INCLUDE=\"/opt/ros/${ROS_DISTRO}/include\"" >> /root/.bashrc && \
    echo "export CYCLONE_LIB=\"/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu\"" >> /root/.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=\"LOCALHOST\"" >> /root/.bashrc && \
    echo "export ROS_STATIC_PEER=\"\"" >> /root/.bashrc

CMD ["bash"]