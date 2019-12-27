# edgetpu_roscpp
Use Edge TPU (Coral) with ROS based on C++

## Install

### Build source
```
$ cd <catkin_ws>
$ wstool init src
$ wstool set -u -t src edgetpu_roscpp http://github.com/tongtybj/edgetpu_roscpp --git
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin build
```

### Install device driver for EdgeTPU Coral
```
$ source <catkin_ws>/devel/setup.bash
$ roscd edgetpu_roscpp
$ sudo ./scripts/edgetpu/runtime/install.sh
```

## Usage:

### classify test image:
```
$ roslaunch edgetpu_roscpp classify_image_test.launch
```

### object detection test:
```
$ roslaunch edgetpu_roscpp object_detection_test.launch
```

### single object detection from video stream:
```
$ roslaunch video_stream_opencv camera.launch video_stream_provider:=`rospack find edgetpu_roscpp`/test/data/DJI_0004.MP4 loop_videofile:=true
$ roslaunch edgetpu_roscpp detection_from_stream.launch verbose:=false image_view:=true
```