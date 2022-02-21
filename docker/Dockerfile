FROM ros:melodic-robot
RUN apt-get update && apt-get install -y sudo build-essential git curl python3-tk ros-melodic-catkin python3-pip libjpeg-dev \
&& curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -

# required for displaying matplotlib plots
ENV DISPLAY :1
# add user with sudo privileges which is not prompted for password
RUN adduser --disabled-password --gecos '' user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR /home/user
ENV PATH="/home/user/.local/bin:${PATH}"

# installing rviz web for visualization
RUN mkdir -p ~/ws/src
RUN cd ~/ws/src && git clone https://github.com/osrf/rvizweb/

# here it gets installed
RUN cd ~/ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/melodic/setup.sh && cd ~/ws && catkin_make install

# copy spot-ros-pkg in install folder so that rvizweb finds it
RUN cd ~/ws/install/share/ && git clone https://github.com/FrancescoRuscelli/spot-ros-pkg

# source workspace for ROS_PACKAGE_PATH
RUN echo "source ~/ws/install/setup.bash" > .bashrc

# break cache for re-installation of casadi (to always download the last version)
ARG CACHE_DATE="date"

# install casadi-horizon
RUN pip3 install casadi-horizon

# overwrite ros launch to enable rvizweb
COPY rviz/launcher.launch /home/user/.local/lib/python3.6/site-packages/horizon/examples/replay/launch/

# add a configuration for the rvizweb (robot model, grid, world...)
COPY configuration.json /home/user/ws/install/share/rvizweb/config/

#COPY examples.sh examples.sh
#CMD sudo chmod +x examples.sh && ./examples.sh