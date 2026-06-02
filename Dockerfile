FROM mxwilliam/mxck:foxy-pytorch-l4t-35.4-core

# Upgrade pip and install Python packages
# RUN python3 -m pip install \
# ...

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...


# Pin NumPy for TensorRT compatibility
RUN python3 -m pip install --no-cache-dir --force-reinstall \
    numpy==1.23.5

RUN python3 -m pip install --no-cache-dir \
    git+https://github.com/william-mx/ros2_pydata.git

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./.bash_aliases /root/.bash_aliases

# Mark all Git repos as safe
RUN git config --global --add safe.directory '*'
