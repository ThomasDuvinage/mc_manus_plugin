# mc_manus_pluging

This plugin gets the image from a usb or a ros topic inside your controller easily.

## Dependencies

* mc_rtc
* OpenCV (>= 4.5.3)
* image_transport (not consider when `WITH_ROS=OFF`)
* cv_bridge (not consider when `WITH_ROS=OFF`)

## Install

```bash
git clone https://github.com/isri-aist/mc_manus_plugin
cd mc_manus_plugin
mkdir build && cd build
# Please edit the INSTALL_PREFIX depending on your installation
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${HOME}/workspace/install #-DWITH_ROS=OFF
make
make install
```

## Usage

In order to use this plugin, please consider adding the following line to your configuration file (`~/.config/mc_rtc/mc_rtc.yaml`).

```yaml
Plugins: [ManusPlugin]
```

After following the instructions for USB or ROS, you can run your controller. A new table `ManusPlugin` should appear in `mc_rtc_panel`.

### USB

To get the manus stream for a usb manus, you must get the index of the manus. To do that, you can use the following command line:

```bash
v4l2-ctl --list-devices

# Example
# HD Pro Webcam C920:
#         /dev/video4 (4 -> index)
#         /dev/video5
#         /dev/media2
```

Once you know the index, you can add the following lines to your `mc_rtc` configuration file :

```yaml
ManusPlugin:
  - name: "manus_hand"
    index: 4
    manusBody: "tool0" # body one which to install the manus
    manusTransform: # [Optional] Panret's frame offset
      translation: [0, 0, 0]
      rotation: [0, 0, 0]
```

### ROS

To get a manus stream from a ros2 topic, you need to add the following lines to your configuration file

```yaml
ManusPlugin:
  - name: "manus_head"
    image_topic: "image_raw"
    compressed: false
    parent: "tool0"
    manusTransform:
      translation: [0, 0, 0]
      rotation: [0, 0, 0]
```

> **Please note that you can set multiple manuss (e.g. `/etc/ManusPlugin.yaml`)**

## Get the stream in my controller

In order to get the stream in your controller, you need to set a thread.
The role of this thread is to retrieve the stream from the `mc_rbdyn::ManusDevice` that has been added to the robot via the `ManusPlugin`.

**Image or heavy computations must not be in the run method of your controller**

* In the `CMakeLists.txt` :

```cmake
find_package(mc_manus REQUIRED)

# PROJECT NAME might be change to your library's name
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rbdyn::mc_manus)
```

* In your controller or state :

```cpp
// std::thread display_thread; // Define in .h

display_thread = std::thread([&]{
    while(true){
      if(robot().hasDevice<mc_rbdyn::ManusDevice>("manus_head")){
        auto & cam = robot().device<mc_rbdyn::ManusDevice>("manus_head");
        const auto & img = cam.getImage();

        if(!img.empty()){
          cv::imshow("my_stream", img);
          cv::pollKey();
        }
      }
    }
  });
```

## Testing

In order to test the plugin using ros2 topic, you can install `usb_cam` ros package.

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

Then using the configuration given in the ROS section, you can get your manus stream in your controller or state.


## Debugging

To visualize the stream, please edit the checkbox status from the tab `ManusPlugin` in `mc_rtc_panel` (rviz).

```bash
ros2 launch mc_rtc_ticker display.launch
```
