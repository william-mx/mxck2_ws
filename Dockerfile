FROM mxwilliam/mxck:core-foxy-pytorch-l4t-35.4

# Upgrade pip and install Python packages
# RUN python3 -m pip install \
# ...

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...


RUN python3 -m pip install --force-reinstall --no-cache-dir \
    git+https://github.com/william-mx/ros2_numpy.git


COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./.bash_aliases /root/.bash_aliases

# Mark all Git repos as safe
RUN git config --global --add safe.directory '*'
