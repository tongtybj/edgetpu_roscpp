# edgetpu_roscpp
Use Edge TPU (Coral) with ROS based on C++

## Usage:

### classify test image:
```
$ rosrun edgetpu_roscpp classify_image --model_path=`rospack find edgetpu_roscpp`/test_data/mobilenet_v1_1.0_224_quant_edgetpu.tflite --image_path=`rospack find edgetpu_roscpp`/test_data/cat.bmp  --labels_path=`rospack find edgetpu_roscpp`/test_data/imagenet_labels.txt
```