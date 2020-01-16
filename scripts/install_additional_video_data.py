#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'edgetpu_roscpp'

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

