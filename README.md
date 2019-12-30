# edgetpu_roscpp
Use Edge TPU (Coral) with ROS based on C++

## Install

### Build source
```
$ cd <catkin_ws>
$ wstool init src
$ wstool set -u -t src edgetpu_roscpp http://github.com/tongtybj/edgetpu_roscpp --git
$ wstool merge -t src src/edgetpu_roscpp/edgetpu_roscpp_${ROS_DISTRO}.rosinstall
$ wstool update -t src
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
$ roslaunch edgetpu_roscpp detection_from_stream.launch verbose:=false image_view:=true
$ roslaunch video_stream_opencv camera.launch video_stream_provider:=`rospack find edgetpu_roscpp`/test/data/DJI_0004.MP4 loop_videofile:=true
$ rqt_image_view /deep_object_detection/detection_result
```

### single object tracking by detection:

#### status color:
- a reversed color in a resulted image: can neither detect nor tracking a target.
- a blue bouding box in a resulted image: detecting a good candidate
- a green bouding box in a resulted image: tracking a good target
- a yellow bouding box in a resulted image: losing the target, and keep the last good tracking result.

#### main parameters:
- `coarse_detection_score_threshold`: in detection phase, we apply coarse-to-refined detection method. This parameter is for the first step detection. 
- `refined_detection_score_threshold`: in detection phase,  we apply coarse-to-refined detection method. This parameter is for the second step detection.
- `tracking_score_threshold`: the detection based tracking score in tracking phase, which is after the detection phase.
- `expanding_bounding_box_rate`: in refined detection step, we expand the bouding box resulted from the coarse detection and crop this expanded area from the raw image for the refined detection. This parameter is the expanding rate.
- `keep_aspect_ratio_in_expanded_bbox`: besides the above `expanding_bounding_box_rate`, we also have to modifiy  the aspect ratio of the crop area for refined detection. Empirically, the same aspect ratio with raw image (e.g. 4:3 for 1080HD, 5:4 for 720HD) gives the best solution so far. This is also associated with the resize process for inference. Please follow the discussion [here](https://github.com/google-coral/edgetpu/issues/36).
- `keep_aspect_ratio_in_inference`: This is the paramter to choose whether keep the aspect ratio of a reszied image for inference. To keep the aspect ratio, we have padding the free space. Empirically, set this parameter as `false` is good for the inference model trained from [here](https://coral.ai/docs/edgetpu/retrain-detection/) or [here](https://github.com/knorth55/73b2_kitchen_edgetpu_object_detection). Please follow the discussion [here](https://github.com/google-coral/edgetpu/issues/36).
. Empirically, the same aspect ratio with raw image (e.g. 4:3 for 1080HD, 5:4 for 720HD) gives the best solution so far. This is also associated with the resize process for inference. Please follow the discussion [here](https://github.com/google-coral/edgetpu/issues/36).
- `detection_check_frame_num`: the frame count to decide a detected candidate is a fixed target. We check whether the motion of the candidate is continous.
- `lost_target_check_frame_num`: the frame count to decide the lost target. During the losing target phase, we will keep the last good tracking result.
- `redetection_after_lost_target_frame_num`: after the lost target, we design a quick redetction process for recovery the tracking mode (i.e., `detection_check_frame_num / 2`). However, if the lost time (frames) is longer than this parameter, we will back a normal detection phase, that is no more quick redetection measure.
- `quick_detection`: if this parameter is `true`, we find all good candidates in different size of images (i.e., a full size image and 5 half size images in top-left, top-right, down-left, down-right and center respectively) in the detection phase, and choose the candiate with the best score as the result in every frame. If this parameter is `false`,  then we only choose the first good candidate which has higher score than the `refined_detection_score_threshold`.

#### from video stream:
```
$ roslaunch edgetpu_roscpp single_object_tracking_by_detection.launch  image_view:=true verbose:=false coarse_detection_score_threshold:=0.0 refined_detection_score_threshold:=0.7  tracking_score_threshold:=0.7  keep_aspect_ratio_in_expanded_bbox:=true keep_aspect_ratio_in_inference:=false quick_detection:=true
$ roslaunch video_stream_opencv camera.launch video_stream_provider:=`rospack find edgetpu_roscpp`/test/data/DJI_0004.MP4
$ rqt_image_view /single_object_detection_and_tracking/detection_result_image
```
**note**: you can also play other video, also add `start_frame` and `stop_frame` to set the start and end point of a video, and `fps` to set the frame rate:
- example 1: start a video from the middle with slower rate 10Hz.
```
$ roslaunch video_stream_opencv camera.launch video_stream_provider:=`rospack find edgetpu_roscpp`/test/data/DJI_0006.MP4 loop_videofile:=true start_frame:=7830 stop_frame:=-1 fps:=10
```

- example 2: publish a single frame from a video with 1 Hz
```
$ roslaunch video_stream_opencv camera.launch video_stream_provider:=/home/leus/drone_detection_dataset/video/DJI_0006.MP4 loop_videofile:=true start_frame:=6620 stop_frame:=6621 fps:=1
```
