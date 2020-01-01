# -*- coding: utf-8 -*-

import argparse
import os
from object_detection.utils import label_map_util

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir')
    parser.add_argument('--ckpt_dir')
    parser.add_argument('--batch_size')
    args = parser.parse_args()

    data_dir = args.data_dir
    ckpt_dir = args.ckpt_dir
    config_path = os.path.join(ckpt_dir, 'pipeline.config')
    label_map_path = os.path.join(data_dir, 'tf_label_map.pbtxt')
    label_map_dict = label_map_util.get_label_map_dict(label_map_path)
    n_class = len(label_map_dict)

    os.system(
        'sed -i "s%CKPT_DIR_TO_CONFIGURE%{0}%g" "{1}"'
        .format(ckpt_dir, config_path))
    os.system(
        'sed -i "s%DATASET_DIR_TO_CONFIGURE%{0}%g" "{1}"'
        .format(data_dir, config_path))
    os.system(
        'sed -i "s%NUM_CLASSES_TO_CONFIGURE%{0}%g" "{1}"'
        .format(n_class, config_path))
    os.system(
        'sed -i "s%NUM_BATCH_SIZE_TO_CONFIGURE%{0}%g" "{1}"'
        .format(args.batch_size, config_path))


if __name__ == '__main__':
    main()
