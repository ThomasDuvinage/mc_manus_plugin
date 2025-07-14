# mc_camera_pluging

This plugin gets the image from a usb or a ros topic inside your controller easily.

## Dependencies

* mc_rtc
* OpenCV (>= 4.5.3)
* image_transport (not consider when `WITH_ROS=OFF`)
* cv_bridge (not consider when `WITH_ROS=OFF`)

## Install

```bash
git clone https://github.com/isri-aist/mc_camera_plugin
cd mc_camera_plugin
mkdir build && cd build
# Please edit the INSTALL_PREFIX depending on your installation
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${HOME}/workspace/install #-DWITH_ROS=OFF
make
make install
```

## Usage

In order to use this plugin, please consider adding the following line to your configuration file (`~/.config/mc_rtc/mc_rtc.yaml`).

```yaml
Plugins: [CameraPlugin]
```

After following the instructions for USB or ROS, you can run your controller. A new table `CameraPlugin` should appear in `mc_rtc_panel`.

### USB

To get the camera stream for a usb camera, you must get the index of the camera. To do that, you can use the following command line:

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
CameraPlugin:
  - name: "camera_hand"
    index: 4
    cameraBody: "tool0" # body one which to install the camera
    cameraTransform: # [Optional] Panret's frame offset
      translation: [0, 0, 0]
      rotation: [0, 0, 0]
```

### ROS

To get a camera stream from a ros2 topic, you need to add the following lines to your configuration file

```yaml
CameraPlugin:
  - name: "camera_head"
    image_topic: "image_raw"
    parent: "tool0"
    cameraTransform:
      translation: [0, 0, 0]
      rotation: [0, 0, 0]
```

> **Please note that you can set multiple cameras (e.g. `/etc/CameraPlugin.yaml`)**

## Get the stream in my controller

In order to get the stream in your controller, you need to set a thread.
The role of this thread is to retrieve the stream from the `mc_rbdyn::CameraDevice` that has been added to the robot via the `CameraPlugin`.

**Image or heavy computations must not be in the run method of your controller**

* In the `CMakeLists.txt` :

```cmake
find_package(mc_camera REQUIRED)

# PROJECT NAME might be change to your library's name
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rbdyn::mc_camera)
```

* In your controller or state :

```cpp
// std::thread display_thread; // Define in .h

display_thread = std::thread([&]{
    while(true){
      if(robot().hasDevice<mc_rbdyn::CameraDevice>("camera_head")){
        auto & cam = robot().device<mc_rbdyn::CameraDevice>("camera_head");
        const auto & img = cam.getImage();

        if(!img.empty()){
          cv::imshow("my_stream", img);
          cv::pollKey();
        }
      }
    }
  });
```

## Debugging

To visualize the stream, please edit the checkbox status from the tab `CameraPlugin` in `mc_rtc_panel` (rviz).

```bash
ros2 launch mc_rtc_ticker display.launch
```
