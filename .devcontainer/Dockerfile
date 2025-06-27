FROM althack/ros2:jazzy-gazebo

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive

# Fix Qt runtime directory issues for GUI applications
RUN mkdir -p /run/user/1000 && chown ros:ros /run/user/1000 && chmod 700 /run/user/1000

# Update package lists to ensure we have the latest ROS packages available
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-rosdep \
   python3-colcon-common-extensions \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
RUN rosdep init || true \
   && rosdep update

# Copy source files and install dependencies
COPY src /tmp/src
RUN apt-get update \
   && rosdep install --from-paths /tmp/src --ignore-src -r -y \
   && rm -rf /tmp/src \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

