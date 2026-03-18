FROM osrf/ros:humble-desktop AS base

ARG USERNAME=orca4
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

# Install a few handy tools
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglapi-mesa \
    libosmesa6 \
    mlocate \
    software-properties-common \
    sudo \
    wget \
    vim \
    libjpeg-dev \ 
    libpng-dev \ 
    catch2 \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libc++-dev \
    libepoxy-dev \
    libglew-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install ros-humble-ros-gz \
  && rm -rf /var/lib/apt/lists/*

# Add packages.osrfoundation.org, required by ardupilot_gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add OSRF's rosdep rules so that rosdep will find Gazebo Harmonic keys
# Ref https://github.com/ArduPilot/ardupilot_gazebo#Rosdep
RUN wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list


# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    gz-garden \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*
  
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install some ardupilot and ardupilot_gazebo prereqs
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    python3-pip \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
    libunwind-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
# Required for ArduSub install, but generally a good idea
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by orca4 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

USER root
WORKDIR /tmp/
RUN git clone --recursive -b v0.9.2 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    # apt update && \
    # ./scripts/install_prerequisites.sh recommended && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    cd .. && \
    rm -rf build


# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME

# Clone ArduSub code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub prereqs (this also appends to .bashrc)
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
# Note: waf will capture all of the environment variables in ardupilot/.lock-waf_linux_build.
# Any change to enviroment variables will cause a re-build.
# To avoid this call sim_vehicle.py with the "--no-rebuild" option.
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

#Clone ORB-SLAM3
WORKDIR /home/$USERNAME
RUN git clone https://github.com/Projeto-Voris/ORB-SLAM3.git ORB_SLAM3 

# Build ORBSLAM 3
WORKDIR /home/$USER/ORB_SLAM3
RUN chmod +x build.sh &&\
    ./build.sh

  # Create colcon workspace and the mount point for orca4
WORKDIR /home/$USERNAME
RUN mkdir -p ros2_ws/src/orca4

  # Copy orca4 package.xml files
WORKDIR /home/$USERNAME/ros2_ws
COPY --chown=$USER_UID:$USER_GID orca_base/package.xml src/orca4/orca_base/
COPY --chown=$USER_UID:$USER_GID orca_bringup/package.xml src/orca4/orca_bringup/
COPY --chown=$USER_UID:$USER_GID orca_description/package.xml src/orca4/orca_description/
COPY --chown=$USER_UID:$USER_GID orca_msgs/package.xml src/orca4/orca_msgs/
COPY --chown=$USER_UID:$USER_GID orca_shared/package.xml src/orca4/orca_shared/

# Run rosdep over orca4 package.xml files
WORKDIR /home/$USERNAME/ros2_ws
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src 
  # --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"



# Build for Gazebo Harmonic, define this before building ardupilot_gazebo and ros_gz
ENV GZ_VERSION=garden
#Get repos
WORKDIR /home/$USERNAME/ros2_ws/src
COPY --chown=$USER_UID:$USER_GID workspace.repos orca4
RUN vcs import --recursive < orca4/workspace.repos
#RUN tar -xf orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt.tar.gz -C src/orbslam3_ros2/orbslam3_ros2/vocabulary


# Build ardupilot_gazebo
USER $USERNAME
# Run rosdep over sim repos
WORKDIR /home/$USERNAME/ros2_ws
RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src

COPY cv_bridge.hpp /opt/ros/humble/include/cv_bridge/cv_bridge


WORKDIR /home/$USERNAME/ros2_ws
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build --packages-select orbslam3_msgs orbslam3_ros2 --symlink-install" ]
  
# RUN pip install --upgrade setuptools==70.2.0 pynput==1.7.7

# Do not build orca5 yet
WORKDIR /home/$USERNAME/ros2_ws
RUN touch src/orca4/COLCON_IGNORE

# Build everything so far
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build --event-handlers console_direct+" ]




#MAVROS depends on GeographicLib, and GeographicLib needs some datasets
RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && sudo ./install_geographiclib_datasets.sh" ]

# Copy everything

# Build everything
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

VOLUME /home/$USERNAME/ros2_ws/src/orca4
# Set up the environment
WORKDIR /home/$USERNAME/ros2_ws
RUN echo "export PATH=/home/$USERNAME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "export PATH=/home/$USERNAME/.local/lib/python3.10/site-packages:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/ros2_ws/src/orca4/setup.bash" >> /home/$USERNAME/.bashrc \
  && echo "source //home/$USERNAME/ros2_ws/install/setup.sh" >> /home/$USERNAME/.bashrc \
  && echo "source /opt/ros/humble/setup.sh" >> /home/$USERNAME/.bashrc \
  && echo "export ROS_DOMAIN_ID=3" >> /home/$USERNAME/.bashrc \
  && echo "alias cw='cd /home/$USERNAME/ros2_ws'" >> /home/$USERNAME/.bashrc \
  && echo "alias cs='cd /home/$USERNAME/ros2_ws/src'" >> /home/$USERNAME/.bashrc \
  && echo "alias cb='cd /home/$USERNAME/ros2_ws && colcon build'" >> /home/$USERNAME/.bashrc
# Required to use the --console option on sim_vehicle.py:
# RUN pip3 install matplotlib
# Use bash as the default shell
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash"]
