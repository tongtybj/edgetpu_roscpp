#!/usr/bin/env bash

docker run --name object-detection-edgetpu \
    --gpus all \
    --rm -it --privileged -p 6006:6006 \
    --mount type=bind,src=${HOME}/object_learn,dst=/tensorflow/models/research/object_learn \
    --mount type=bind,src=$(rospack find edgetpu_roscpp)/training/scripts,dst=/tensorflow/models/research/train_data_scripts \
    object-detection${1}
