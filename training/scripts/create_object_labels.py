# -*- coding: utf-8 -*-

import argparse
import os
from object_detection.utils import label_map_util

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir')
    parser.add_argument('--output_dir')
    args = parser.parse_args()

    print (args.data_dir)
    label_map_path = os.path.join(args.data_dir, 'tf_label_map.pbtxt')
    class_dict = label_map_util.get_label_map_dict(label_map_path)

    text = u""
    for i, name in enumerate(class_dict):
        print i, name
        txt = u"""{0} {1}""".format(i, name)
        text = text + txt
    output_path = os.path.join(args.output_dir, 'tf_labels.txt')
    with open(output_path, 'w') as f:
        f.write(text)

if __name__ == '__main__':
    main()
