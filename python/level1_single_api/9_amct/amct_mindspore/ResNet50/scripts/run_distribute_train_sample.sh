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



if [ $# != 6 ]
then
	echo "Usage: bash run_distribute_train_sample.sh [resnet50] [cifar10] [RANK_TABLE_FILE] [TRAIN_DATASET_PATH] [EVAL_DATASET_PATH] [PRETRAINED_CKPT_PATH]"
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

if [ ! -f $PATH1 ]
then
    echo "error: RANK_TABLE_FILE=$PATH1 is not a file"
exit 1
fi

if [ ! -d $PATH2 ]
then
    echo "error: TRAIN_DATASET_PATH=$PATH2 is not a directory"
exit 1
fi

if [ ! -d $PATH3 ]
then
    echo "error: EVAL_DATASET_PATH=$PATH2 is not a directory"
exit 1
fi

if [ $# == 5 ] && [ ! -f $PATH4 ]
then
    echo "error: PRETRAINED_CKPT_PATH=$PATH4 is not a file"
exit 1
fi

# do the calibration using one device to get the scale_offset_record file
export DUMP_AMCT_RECORD=1 && cd ../ && python3 resnet50_sample.py --dataset_path=$PATH3 --checkpoint_path=$PATH4
cd ./scripts

ulimit -u unlimited
export DEVICE_NUM=8
export RANK_SIZE=8
export RANK_TABLE_FILE=$PATH1
export MINDSPORE_HCCL_CONFIG_PATH=$PATH1
export SERVER_ID=0
rank_start=$((DEVICE_NUM * SERVER_ID))

for((i=0; i<${DEVICE_NUM}; i++))
do
    export DEVICE_ID=${i}
    export RANK_ID=$((rank_start + i))
    rm -rf ./train_parallel$i
    mkdir ./train_parallel$i
    cp ../src/*.py ./train_parallel$i
    cp *.sh ./train_parallel$i
    cp -r ../src ./train_parallel$i
    cp -r ../amct_dump ./train_parallel$i
    cd ./train_parallel$i || exit
    echo "start training for rank $RANK_ID, device $DEVICE_ID"
    env > env.log

    if [ $# == 6 ]
    then
        python3 resnet50_retrain_sample.py \
            --net=$1 \
            --dataset=$2 \
            --run_distribute=True \
            --device_num=$DEVICE_NUM \
            --train_dataset=$PATH2 \
            --pre_trained=$PATH4 &> log &
    fi

    cd ..
done
