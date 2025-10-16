# mc_manus_plugin

This plugin gets the data of the [Manus gloves](https://www.manus-meta.com/products/quantum-metagloves) using the tools of Manus gloves for the C++ SDK of Linux you could look the documentation in the official [web site](https://docs.manus-meta.com/3.0.0/Plugins/SDK/Linux/), it get the ros topics inside of the controller easily.



## Dependencies

* mc_rtc
* ROS2

## Prerequisites to install

#### Installing required packages with a single line sudo: Installing required packages
```bash
sudo apt-get update && sudo apt-get install -y build-essential git libtool libzmq3-dev libusb-1.0-0-dev zlib1g-dev libudev-dev gdb libncurses5-dev && sudo apt-get clean
```

#### Clone the GRPC repository and all its submodules from GitHub to your local machine using the following command:
```bash
sudo git clone -b v1.28.1 https://github.com/grpc/grpc /var/local/git/grpc && cd /var/local/git/grpc && sudo git submodule update --init --recursive
```

#### Install the protobuf dependency by running the following command:
```bash
cd /var/local/git/grpc/third_party/protobuf && sudo ./autogen.sh && sudo ./configure --enable-shared && sudo make -j$(nproc) && sudo make -j$(nproc) check && sudo make install && sudo make clean && sudo ldconfig
```
#### Finally install GRPC by running the command below:
```bash
cd /var/local/git/grpc && sudo make -j$(nproc) && sudo make install && sudo make clean && sudo ldconfig
```

#### Core Integrated
```bash
sudo apt-get update && sudo apt-get install -y build-essential libusb-1.0-0-dev zlib1g-dev libudev-dev gdb libncurses5-dev && sudo apt-get clean
```

#### Device rules: To allow connections to MANUS hardware you need to place the following file in the etc/udev/rules.d/ directory. This will allow the devices to be recognized and accessed by the system. After doing this, a full reboot is recommended to apply the changes. The naming of the file is relevant we recommend naming it 70-manus-hid.rules.

  ##### 70-manus-hid.rules

```bash
# HIDAPI/libusb
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"

# HIDAPI/hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
```

#### Install the packages of manus

To use the Manus Core 3 you need to download the [MANUS Core 3 SDK (including ROS2 Package)](https://docs.manus-meta.com/latest/Resources/)

And you have to click in this option:
![MANUS Core 3 SDK including ROS2 Package](images/manusDownload.png)

This will give you a .zip that you will have to put in your workspace.
The File that you have to focus on is, SDKClient_Linux



#### Docker


Instead of setting up a linux machine, you can also use docker to set up your development environment. Dockerfiles for both **integrated** and **remote**. For this implementation you should use the **integrated** mode, so you could have everything in linux environment.

#### Build integrated docker image
```bash
docker build -f ./Dockerfile.Integrated -t manus-linux-integrated .
```
Once the Docker image is built, run a container with the manus-linux-integrated image. The --net=host is not strictly necessary but helps the container communicate out of its context. The --privileged parameter is required to access the USB glove devices, so is mounting the /dev and /run/udev directories.

#### Run integrated docker container
```bash
docker run -p 5000:5000 --privileged -v /dev:/dev -v /run/udev:/run/udev -i -t manus-linux-integrated /bin/bash
```



## Install


```bash
git clone https://github.com/Hugo734/mc_manus_plugin
cd mc_manus_plugin
mkdir build && cd build
# Please edit the INSTALL_PREFIX depending on your installation
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${HOME}/workspace/install #-DWITH_ROS=OFF
make
make install
```
## Compilation
For compiling the examples we have three diffrent options.

```bash
1. Make
2. Visual Studio Code
3. Visual Studio (remote cross compile)
```

## Make
A Makefile is present within the SDKMinimalCLient_Linux and SDKClient_Linux folders inside the MANUS SDK package. The example can be compiled using the following command.

```bash
make all
```

## Usage

In order to use this plugin, please consider adding the following line to your configuration file (`~/.config/mc_rtc/mc_rtc.yaml`).

Also you need to run the

```yaml
Plugins: [ManusPlugin]
```

After following the instructions for USB or ROS, you can run your controller. A new table `ManusPlugin` should appear in `mc_rtc_panel`.


```bash
---
ManusPlugin:
  - name: "right_hand"
    topic: "/manus_glove_0"

  - name: "left_hand"
    topic: "/manus_glove_1"
```

## Get the information

To get the device info please consider adding the following line to your controller :

```cpp
if(gc.controller().robot().hasDevice<mc_rbdyn::ManusDevice>("leaf_hand"))
    {
      auto & manus_glove = gc.controller().robot().device<mc_rbdyn::ManusDevice>("left_hand");
      const auto & manus_info = manus_glove.data();
      std::cout << manus_info.gloveId << std::endl;
    }
```

  Each entry creates an mc_rbdyn::ManusDevice attached to parent. Run your controller and a ManusPlugin tab will appear in mc_rtc_panel.

> **Please note that you can set multiple manuss (e.g. `/etc/ManusPlugin.yaml`)**
