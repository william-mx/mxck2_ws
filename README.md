# MXcarkit ROS2 Foxy Development Workspace

Clone this repository.
```
git clone --recurse-submodules -b development_ws https://github.com/william-mx/mxck2_ws.git ~/development_ws
cd ~/development_ws
git submodule update --remote --merge --recursive
```

## Choosing the Right CUDA Docker on Jetson (L4T)

On Jetson devices, **L4T (Linux for Tegra)** already includes the **NVIDIA GPU kernel driver** and **CUDA driver libraries** (e.g., `nvgpu.ko`, `libcuda.so`). This means you **do not need to install the CUDA runtime via the SDK Manager** if you plan to use CUDA **inside a Docker container** rather than directly on the host.  

To access the GPU from within a container, the **NVIDIA Container Runtime** bridges the container with the host’s GPU driver.  
Inside the container, you have the **CUDA user-space toolkit** (e.g., `libcudart`, `cuDNN`, `TensorRT`).


⚠️ **Important:** The container’s CUDA version must **match the host’s CUDA driver version**, otherwise CUDA programs may fail to run.

Each **JetPack / L4T** release corresponds to one CUDA base version:

| JetPack / L4T | CUDA Version | Ubuntu Base | ROS 2 |
|----------------|--------------|--------------|-------------------|
| 4.x (R32.x) | 10.2 | Ubuntu 18.04 | Dashing |
| 5.x (R35.x) | 11.4 | Ubuntu 20.04 | Foxy |
| 6.x (R36.x) | 12.6 | Ubuntu 22.04 | Humble |


Since **Ultralytics** provides simple, high-performance Docker images for vision models,  
we recommend using their **Jetson optimized images** as a base, available here:  [Ultralytics on Docker Hub](https://hub.docker.com/r/ultralytics/ultralytics)

## Test if everything is set up correctly

You can verify that CUDA and the GPU driver are working with the built-in `deviceQuery` sample:

```bash
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
make -j"$(nproc)" && ./deviceQuery
```

**What this does:**

* Compiles a small CUDA test program (`deviceQuery`) using the local CUDA toolkit.
* Runs it to query GPU properties (name, compute capability, memory, etc.).
* If the setup is correct, it prints details like:

```
Detected 1 CUDA Capable device(s)
Device 0: "Xavier"
CUDA Driver Version / Runtime Version: 11.4 / 11.4
```

