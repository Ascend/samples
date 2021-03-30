#!/bin/bash

# Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the Apache License Version 2.0.You may not use
# this file except in compliance with the License.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# Apache License for more details at
# http://www.apache.org/licenses/LICENSE-2.0



if [ $# != 5 ]
then
	echo "Usage: bash run_standalone_train_sample.sh [resnet50] [cifar10] [TRAIN_DATASET_PATH] [EVAL_DATASET_PATH] [PRETRAINED_CKPT_PATH]"
exit 1
fi

if [ $1 != "resnet50" ]
then
    echo "error: the selected net is not resnet50"
exit 1
fi

if [ $2 != "cifar10" ]
then
    echo "error: the selected dataset is not cifar10"
exit 1
fi


get_real_path(){
  if [ "${1:0:1}" == "/" ]; then
    echo "$1"
  else
    echo "$(realpath -m $PWD/$1)"
  fi
}

PATH1=$(get_real_path $3)
PATH2=$(get_real_path $4)
PATH3=$(get_real_path $5)

if [ $# == 6 ]
then
    PATH4=$(get_real_path $6)
fi


if [ ! -d $PATH1 ]
then
    echo "error: TRAIN_DATASET_PATH=$PATH1 is not a directory"
exit 1
fi

if [ ! -d $PATH2 ]
then
    echo "error: EVAL_DATASET_PATH=$PATH2 is not a directory"
exit 1
fi

if [ $# == 5 ] && [ ! -f $PATH3 ]
then
    echo "error: PRETRAINED_CKPT_PATH=$PATH3 is not a file"
exit 1
fi

ulimit -u unlimited
export DEVICE_NUM=1
export DEVICE_ID=0
export RANK_ID=0
export RANK_SIZE=1

if [ -d "train" ];
then
    rm -rf ./train
fi
mkdir ./train
cp ../src/*.py ./train
cp *.sh ./train
cp -r ../src ./train
cd ./train || exit
echo "start training for device $DEVICE_ID"
env > env.log


if [ $# == 5 ]
then
    python3 resnet50_retrain_sample.py --net=$1 \
        --dataset=$2 \
        --run_distribute=False \
        --device_num=$DEVICE_NUM \
        --train_dataset=$PATH1 \
        --eval_dataset=$PATH2 \
        --pre_trained=$PATH3 | tee log

fi
cd ..

