# traininig model for edgetpu

## general instruction:

- Overview: https://coral.ai/docs/edgetpu/models-intro/
- Retrain an object detection model: https://coral.ai/docs/edgetpu/retrain-detection/

## annotation

### VoTT

URL: https://github.com/knorth55/73b2_kitchen_edgetpu_object_detection

#### install
1. install NodeJS (>= 10.x, Dubnium) and NPM. check [this instruction](https://qiita.com/seibe/items/36cef7df85fe2cefa3ea)
```
$ sudo apt install -y nodejs npm
$ sudo npm install n -g
$ sudo n 11.5.0
$ node -v # should be v11.5.0
```
2. build VoTT
Please follow  [Build and run from source](https://github.com/microsoft/VoTT#build-and-run-from-source).
```
$ git clone https://github.com/Microsoft/VoTT.git
$ cd VoTT
$ npm ci
```

3. start
```
$ npm start
```

    Then check [Using VoTT](https://github.com/microsoft/VoTT#using-vott):
        1. create connections
           [images]
           **note**: connection means the dataset. Sellect the local file path of the directory of training data set (e.g, raw images or videos). For example, `${HOME}/object_learn/videos`
        2. creating a New Project
           [images]
           **note**: `target connection` is the directory to save the project and export the labelled images. For example, `${HOME}/object_learn/annotation`.
        3. export settings
           [images]
           **note**: `Provider`: `Tensorflow Records`; `Asset State`: `Only tagged Assets`.
        4. share the project in other PC (under development)
           you have to check the security token as mentioned in [security-tokens](https://github.com/microsoft/VoTT#security-tokens).

5. export the project result

### prepare for the training dataset (Deprecated)
1. use python3 (> 3.6) in virtualenv in host pc:
```
$ sudo apt install python-virtualenv
$ cd ~ && mkdir -p python-venv && cd python-venv
$ virtualenv -p python3.6 venv-py3-6
$ source venv-py3-6/bin/activate
```

2. install `split-folders`:
```
(venv-python3-6)$ pip install split-folders tqdm
```

3. use [split-folders](https://pypi.org/project/split-folders/) to create directories for training and validation:
```
(venv-python3-6)$ cd ${HOME}/object_learn/train_data
(venv-python3-6)$ mkdir input/class1
(venv-python3-6)$ ln -nfs ${HOME}/object_learn/annotation/<vott_project_name>-TFRecords-export/*.tfrecord ${HOME}/object_learn/train_data/input/class1
(venv-python3-6)$ split_folders ${HOME}/drone_learn/train_data/input --ratio .9 .1
(venv-python3-6)$ cp ${HOME}/object_learn/annotation/<vott_project_name>-TFRecords-export/tf_label_map.pbtxt ${HOME}/object_learn/train_data/output/
```
**note**: you can do `(venv-python3-6)$ deactivate` to stop the virtualvenv.

## traninig in docker
1. build docker file and start docker
```
$ roscd edgetpu_roscpp/training/ # with gpu
$ docker build . --tag object-detection
$ bash run.bash
```

### inside docker 
2. prepare checkpoint and dataset 
```
$ cd train_data_scripts
$ ./prepare_checkpoint_and_dataset.sh --train_whole_model false --network_type mobilenet_v2_ssd
```

3. start retraininig (last few layers)
- without GPU (multi-core in CPU)
  ```
  $ CUDA_VISIBLE_DEVICES=-1 ./retrain_detection_model.sh --num_training_steps 500 --num_eval_steps 100
  ```

- with single GPU
  ```
  $ CUDA_VISIBLE_DEVICES=0 ./retrain_detection_model.sh --num_training_steps 500 --num_eval_steps 100
  ```

**note**: if you want to retrain the whole model, please follow `Whole retraining in docker` in [this README.txt](https://github.com/tongtybj/73b2_kitchen_edgetpu_object_detection)


### check the training process:

1. in a new terminal of host pc:
   ```
   $ docker exec -it object-detection-edgetpu /bin/bash
   ```
2. then, get into docker:
   ```
   $ tensorboard --logdir=./object_learn/train_data/train 
   ```
3. finally, back the host pc, use any webbrouser, and type `localhost:6006`.

## convert to edgetpu compatible model

1. install in host pc (only onetime)
   ```
   $ curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
   $ echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
   $ sudo apt-get update
   $ sudo apt-get install edgetpu
   ```

2. convert forezen graph into tflite model. Inside docker:
   ```
   $ ./convert_checkpoint_to_edgetpu_tflite.sh --checkpoint_num 500
   ```
3. in host pc:
   ```
   $ cd ${Home}/object_learn/train_data
   $ sudo chmod 777 models
   $ cd models
   $ sudo chmod 755 *
   $ edgetpu_compiler output_tflite_graph.tflite
   ```


## other tips:
- Docker + Nvidia:

    - install nvidia driver for host PC (install CUDA is recommended)
    - install docker CE: https://qiita.com/gen10nal/items/1e7fe8a1b2e9ad1e7919

    - install nvidia-docker2: https://qiita.com/gen10nal/items/1e7fe8a1b2e9ad1e7919

- useful sites:

    - simple tutorial for inference and tranining in Japanese: https://github.com/knorth55/73b2_kitchen_edgetpu_object_detection
    - tranining in docker with GPU: https://github.com/knorth55/73b2_kitchen_edgetpu_object_detection

