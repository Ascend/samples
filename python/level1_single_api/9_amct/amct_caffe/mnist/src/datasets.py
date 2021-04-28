#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

Read data from lmdb dataset, and do some preprocess.
"""
import os
from pathlib import Path
import numpy as np
import caffe # pylint: disable=E0401
import lmdb # pylint: disable=E0401


class LMDBData(object):
    """lmdb dataset reader"""
    def __init__(self, lmdb_dir):
        """init lmdb dataset reader"""
        real_lmdb_dir = os.path.realpath(lmdb_dir)
        if not Path(real_lmdb_dir).exists():
            raise RuntimeError('Cannot find lmdb dir:{}'.format(lmdb_dir))
        self.__get_shape_from_lmdb(real_lmdb_dir)
        self.__scale = 1
        self.__mean = np.zeros((self.__channels, self.__height,
                                self.__width), np.float32)
        lmdb_env = lmdb.open(real_lmdb_dir)
        lmdb_txn = lmdb_env.begin()
        self.__lmdb_cursor = lmdb_txn.cursor()
        self.__lmdb_cursor.first()
        self.__crop_size = None

    def __get_shape_from_lmdb(self, lmdb_dir):
        """get shape from lmdb"""
        lmdb_env = lmdb.open(lmdb_dir)
        lmdb_txn = lmdb_env.begin()
        lmdb_cursor = lmdb_txn.cursor()
        lmdb_cursor.first()
        _, value = lmdb_cursor.item()
        datum = caffe.proto.caffe_pb2.Datum()
        datum.ParseFromString(value)
        self.__channels = datum.channels
        self.__height = datum.height
        self.__width = datum.width

    def set_scale(self, scale):
        """get scale data"""
        self.__scale = scale

    def set_mean_file(self, mean_file):
        """set mean file to reader"""
        proto_file = os.path.realpath(mean_file)
        if not Path(proto_file).exists():
            raise RuntimeError('Cannot find mean file:{}'.format(mean_file))
        mean_blobs = caffe.proto.caffe_pb2.BlobProto()
        with open(proto_file, 'rb') as file:
            try:
                pbtxt_string = file.read()
                mean_blobs.ParseFromString(pbtxt_string)
            except Exception as exception:
                raise RuntimeError('Read Blobs proto from binary file:{} ' \
                    'failed, {}'.format(proto_file, exception))
        if mean_blobs.channels != self.__channels:
            raise RuntimeError('Channels in mean file {} is not equal to ' \
                'lmdb data {}'.format(mean_blobs.channels, self.__channels))
        if mean_blobs.height != self.__height:
            raise RuntimeError('Height in mean file {} is not equal to ' \
                'lmdb data {}'.format(mean_blobs.height, self.__height))
        if mean_blobs.width != self.__width:
            raise RuntimeError('Width in mean file {} is not equal to ' \
                'lmdb data {}'.format(mean_blobs.width, self.__width))
        self.__mean = np.array(mean_blobs.data, np.float32)
        self.__mean.shape = (self.__channels, self.__height, self.__width)

    def set_mean_value(self, mean_value):
        """set mean value to reader"""
        if len(mean_value) != self.__channels:
            raise RuntimeError('Mean value must be channel wise!')
        for index, value in enumerate(mean_value):
            self.__mean[index, :, :] = value

    def __check_datum_shape(self, datnum):
        """check data shape"""
        if datnum.channels != self.__channels:
            raise RuntimeError('Channels in datnum file {} is not equal ' \
                'to lmdb data {}'.format(datnum.channels, self.__channels))
        if datnum.height != self.__height:
            raise RuntimeError('Height in datnum file {} is not equal to ' \
                'lmdb data {}'.format(datnum.height, self.__height))
        if datnum.width != self.__width:
            raise RuntimeError('Width in datnum file {} is not equal to ' \
                'lmdb data {}'.format(datnum.width, self.__width))

    def set_crop_size(self, crop_size):
        """set crop size to reader"""
        self.__crop_size = crop_size

    def __crop_operation(self, array):
        """do data crop operation"""
        height = array.shape[1]
        width = array.shape[2]
        if height < self.__crop_size or width < self.__crop_size:
            raise RuntimeError('Crop size must less equal than data size')
        h_off = int((height - self.__crop_size) / 2)
        w_off = int((width - self.__crop_size) / 2)
        crop_data = array[:, h_off: (self.__crop_size + h_off), w_off: (self.__crop_size + w_off)]
        return crop_data

    def get_blobs(self, batch_size):
        """get blobs from lmdb dataset"""
        if self.__crop_size is not None:
            blobs = np.zeros((batch_size, self.__channels, self.__crop_size,
                              self.__crop_size), np.float32)
        else:
            blobs = np.zeros((batch_size, self.__channels, self.__height,
                              self.__width), np.float32)
        labels = np.zeros((batch_size), np.float32)

        for index in range(batch_size):
            _, value = self.__lmdb_cursor.item()
            datum = caffe.proto.caffe_pb2.Datum()
            datum.ParseFromString(value)
            self.__check_datum_shape(datum)
            labels[index] = datum.label
            pixel_datas = np.fromstring(datum.data, np.uint8)
            pixel_datas.shape = (self.__channels, self.__height, self.__width)
            pixel_datas = (pixel_datas - self.__mean) * self.__scale
            # then do crop operation
            if self.__crop_size is not None:
                pixel_datas = self.__crop_operation(pixel_datas)
            blobs[index, :, :, :] = pixel_datas

            if not self.__lmdb_cursor.next():
                print('Restarting data prefetching from start.')
                self.__lmdb_cursor.first()

        return blobs, labels
