# How to use this repository 
First of all, this code has to run on Ubuntu Jammy 22.04 and you need to install ROS2 humble. 
To install the latter have a look here: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Having this setup, you first need to create the following folder structure *random_name/src/*. Inside the *src* you clone 
this repository.
```bash
    git clone https://github.com/riccardofeingold/realsense_normal_plane_computation.git .
```

Now, go one level up using *cd ../*. You should be now inside the *random_name* folder.
At this point, use colcon to build files:
```bash
    colcon build
```
Before you can use ROS2 to run the node you always have to source the setup.bash file first:
```bash
    . install/setup.bash
```

Now you can run the node using:
```bash
    ros2 run magnecko_realsense_node magnecko_realsense_node
```
# librealsense installation guide

Created: March 18, 2023 11:33 AM
Theme: Software

**Make Ubuntu Up-to-date:**

- Update Ubuntu distribution, including getting the latest stable kernel:
    
    ```bash
    sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
    ```
    

****Navigate to magnecko_realsense/librealsense/**** 

- Make sure that no intel realsense is connected for the next steps.
- Install the core packages required to build *librealsense*
binaries and the affected kernel modules:
    
    ```bash
    sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
    ```
    
- Run Intel Realsense permissions script from librealsense root directory:
    
    ```bash
    sudo ./scripts/setup_udev_rules.sh
    ```
    
    - *Notice: One can always remove permissions by running:`./scripts/setup_udev_rules.sh --uninstall`*
    

**Building librealsense2 SDK**

- Navigate to *librealsense* root directory and run `mkdir build && cd build`
- Run CMake:
    - `cmake ../ -DBUILD_EXAMPLES=true` - Builds *librealsense* along with the demos and tutorials
- Recompile and install *librealsense* binaries:`sudo make uninstall && make clean && make && sudo make install`
The shared object will be installed in `/usr/local/lib`, header files in `/usr/local/include`.
The binary demos, tutorials and test files will be copied into `/usr/local/bin`