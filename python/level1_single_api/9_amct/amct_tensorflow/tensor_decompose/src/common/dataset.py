"""
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""


import os
import socket

import numpy as np

from .tf_wrap import tf


class Dataset(object):
    """Container class for a dataset."""

    def __init__(self, images, labels, batch_size, shuffle=True, seed=None):
        """
        Construct a Dataset.

        Args:
            images: Image data.
            labels: Label data.
            batch_size: Batch size.
            shuffle: Whether to shuffle data or not.
            seed: Random seed.

        Raises:
            ValueError: Wrong input arguments.
        """

        if seed is not None:
            np.random.seed(seed)
        if images.shape[0] != labels.shape[0]:
            raise ValueError('Image and labels should have the same size')
        if batch_size > images.shape[0]:
            raise ValueError('Batch size should not exceed image number')
        self._len = images.shape[0]
        self._images = images
        self._labels = labels
        self._index = 0
        self._batch_size = batch_size
        self._shuffle = shuffle
        if shuffle:
            self._shuffle_data()

    def _shuffle_data(self):
        """Shuffle all data once."""
        perm = np.arange(self._len)
        np.random.shuffle(perm)
        self._images = self._images[perm]
        self._labels = self._labels[perm]

    def next_batch(self):
        """
        Get next batch of data.

        Returns:
            (images, labels): A batch of images and labels.
        """
        start = self._index
        if start + self._batch_size > self._len:
            last_num = self._len - start
            images_last = self._images[start:self._len]
            labels_last = self._labels[start:self._len]
            if self._shuffle:
                self._shuffle_data()
            start = 0
            self._index = self._batch_size - last_num
            end = self._index
            images_front = self._images[start:end]
            labels_front = self._labels[start:end]
            return (np.concatenate((images_last, images_front), axis=0),
                    np.concatenate((labels_last, labels_front), axis=0))
        else:
            self._index += self._batch_size
            end = self._index
            return self._images[start:end], self._labels[start:end]


class Mnist(object):
    """MNIST dataset."""

    IMAGE_WIDTH = 28
    IMAGE_HEIGHT = 28
    CLASSES = 10

    @staticmethod
    def load(data_path, val_num=10000, one_hot=True):
        """
        Prepare MNIST data.

        Args:
            data_path: MNIST data path.
            val_num: Number of samples to split from data for validation.
            one_hot: Whether to use ont-hot label or not.

        Returns:
            ((image_train, label_train), (image_val, label_val)): Images and
                labels for training, images and labels for validation.
        """
        socket.setdefaulttimeout(5)  # data download time limit
        data_path = os.path.realpath(data_path)
        if not data_path.endswith('mnist.npz'):
            data_path = os.path.join(data_path, 'mnist.npz')
        # download if not exists
        os.makedirs(os.path.dirname(data_path), exist_ok=True)
        (image_train, label_train), (_, _) = \
            tf.keras.datasets.mnist.load_data(data_path)
        image_train = image_train.astype(np.float32) / 255.0  # normalize
        image_train = np.expand_dims(image_train, -1)
        if one_hot:
            label_train = np.eye(Mnist.CLASSES)[label_train]
        image_val = image_train[-val_num:]
        label_val = label_train[-val_num:]
        image_train = image_train[:-val_num]
        label_train = label_train[:-val_num]
        return (image_train, label_train), (image_val, label_val)
