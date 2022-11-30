#!/bin/bash
python3 -m demo.Demo --confidence-threshold 0.5 --output output \
  --config-file configs/Custom-detection/faster_rcnn_R_101_FPN_base1_jinxiang.yaml \
  --input 0000.jpg 0091.jpg 0085.jpg \
  --opts MODEL.WEIGHTS /home/ma-user/work/checkpoints/voc/faster_rcnn/faster_rcnn_R_101_FPN_base1_jinxiang/model_final.pth
  