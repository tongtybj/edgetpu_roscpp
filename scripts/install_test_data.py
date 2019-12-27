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

    download_data(
        pkg_name=PKG,
        path='test/data/DJI_0006.MP4',
        url='https://drive.google.com/uc?id=1Rd8QV9YgGbdHFz-Y93AYd395itJ8zSvH',
        md5='06ba236e01a75d7a769af623aa44fdff',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='test/data/DJI_0008.MP4',
        url='https://drive.google.com/uc?id=1OitkTpZBWzrae3QKXHDSddPYckM6DrUD',
        md5='12bec3d3840a8da0a98007af1f079b40',
        extract=False,
    )

if __name__ == '__main__':
    main()

