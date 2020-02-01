#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'edgetpu_roscpp'

    download_data(
        pkg_name=PKG,
        path='test/data/test_image.tar.gz',
        url='https://drive.google.com/uc?id=1fYVPEbfzrUdfdRG4WRADVmcoiVngmkuV',
        md5='5ecb3207e6d7ced966ad7921b97d6a70',
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='test/data/drone_detection_20190428.tar.gz',
        url='https://drive.google.com/uc?id=1t6PbEJRMGvig42oI3LNqkSfsSDNxfSgC',
        md5='c8b1705ea4a4300bbfda0e9eddaa6976',
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='test/data/DJI_0004.MP4',
        url='https://drive.google.com/uc?id=160h0GTZUGVFL38Bef3qKBsFI7sjwZMW4',
        md5='cb16ccbf06ecd279918d77329705edd6',
        extract=False,
    )

    # co-compile two model (same content)
    download_data(
        pkg_name=PKG,
        path='test/data/drone_detection_co_compile1_edgetpu.tflite',
        url='https://drive.google.com/uc?id=1SeC_TnsOAHUrqnAMLdXDPJaX-4VD5t4z',
        md5='37a10015390a5a33a21daa11e6b8f712',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='test/data/drone_detection_co_compile2_edgetpu.tflite',
        url='https://drive.google.com/uc?id=140othb2fmY0B-fK_EDG5pIda8lqgT7UW',
        md5='5f7eca3309f0e21df8e4e368575342c1',
        extract=False,
    )

if __name__ == '__main__':
    main()

