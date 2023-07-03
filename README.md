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