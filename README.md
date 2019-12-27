# edgetpu_roscpp
Use Edge TPU (Coral) with ROS based on C++

## Usage:

### classify test image:
```
$ rosrun edgetpu_roscpp classify_image --model_path=`rospack find edgetpu_roscpp`/test_data/mobilenet_v1_1.0_224_quant_edgetpu.tflite --image_path=`rospack find edgetpu_roscpp`/test_data/cat.bmp  --labels_path=`rospack find edgetpu_roscpp`/test_data/imagenet_labels.txt
```

### object detection test:
```
$ rosrun edgetpu_roscpp object_detection --model_path=/home/chou/edgetpu/test_data/ssd_mobilenet_v1_fine_tuned_edgetpu.tflite --image_path=/home/chou/edgetpu/test_data/pets.jpg --labels_path=/home/chou/edgetpu/test_data/pet_labels.txt
```