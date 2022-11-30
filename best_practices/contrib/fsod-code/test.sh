#!/bin/bash

# Test Custom Datasets
python3 -m tools.test_net --num-gpus 1 \
        --config-file configs/Custom-detection/faster_rcnn_R_101_FPN_base1_jinxiang.yaml \
        --eval-only \
        --opts AMP 1 OPT_LEVEL O1 LOSS_SCALE_VALUE 128 MODEL.DEVICE npu:0 SOLVER.IMS_PER_BATCH 1