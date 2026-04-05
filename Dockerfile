FROM mxwilliam/mxck:mxck-humble-ubuntu-22.04

# The base image was built with a ROS apt key that may have since rotated.
# This block removes the stale key and repo file, downloads the current key
# from the official ROS repository, and registers a new repo entry pointing
# to it. Without this, apt cannot authenticate ROS packages and installs fail.
RUN \
    # Remove the outdated repo entry that points to the stale key
    rm -f /etc/apt/sources.list.d/ros2-latest.list \
    # Remove the stale key itself
    && rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    # Download the current ROS signing key from the official ROS repository
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    # Register the ROS apt repository and link it to the fresh key.
    # arch= restricts downloads to this machine's CPU architecture (e.g. arm64).
    # signed-by= tells apt which key to use to verify packages from this repo.
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu jammy main" \
         | tee /etc/apt/sources.list.d/ros2.list > /dev/null
         
# Upgrade pip and install Python packages
# RUN python3 -m pip install \
# ...

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...

RUN python3 -m pip install --no-cache-dir \
    git+https://github.com/william-mx/ros2_pydata.git

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./.bash_aliases /root/.bash_aliases