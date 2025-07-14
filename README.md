# mc_camera_pluging

This plugin helps to get the image from a usb or a ros topic inside your controller.

## Dependencies



## Install


## Usage

### USB

To get the camera stream for a usb camera, you must get the index of the camera. To do that, you can use the following command line:

```bash
v4l2-ctl --list-devices
```

```yaml
CameraPlugin:
  - name: "camera_hand"
    index: 4
    cameraBody: "tool0" # body one which to install the camera
```

### ROS
