#!/bin/bash


# 训练金相数据集
python3 -m tools.train_net --num-gpus 1 \
    --config-file ./configs/Custom-detection/faster_rcnn_R_101_FPN_base1_jinxiang.yaml \
    --opts AMP 1 OPT_LEVEL O1 LOSS_SCALE_VALUE 128 MODEL.DEVICE npu:0 \
    SOLVER.IMS_PER_BATCH 8 SOLVER.MAX_ITER 1800 \
    DATALOADER.NUM_WORKERS 8


# 训练 POSCAL-VOC数据集
# python3 -m tools.train_net --num-gpus 1 \
#     --config-file ./configs/PascalVOC-detection/split1/faster_rcnn_R_101_FPN_base1.yaml \
#     --opts AMP 1 OPT_LEVEL O1 LOSS_SCALE_VALUE 128 MODEL.DEVICE npu:0 \
#     SOLVER.IMS_PER_BATCH 8 SOLVER.MAX_ITER 8000


# 进行实验
# python3 -m tools.run_experiments --num-gpus 1 \
#         --shots 5  --seeds 1 2 --split 1 \


# Reference
# python3 ../tools/train_net.py \
# 	--config-file ../configs/COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml \
# 	AMP 1 \
# 	OPT_LEVEL O2 \
# 	LOSS_SCALE_VALUE 128 \
# 	MODEL.DEVICE npu:$ASCEND_DEVICE_ID \
# 	MODEL.WEIGHTS "$data_path/R-50.pkl" \
# 	MODEL.RPN.NMS_THRESH 0.7 \
# 	MODEL.ROI_BOX_HEAD.POOLER_SAMPLING_RATIO 2 \
# 	MODEL.ROI_MASK_HEAD.POOLER_SAMPLING_RATIO 2 \
# 	DATALOADER.NUM_WORKERS 8 \
# 	SOLVER.IMS_PER_BATCH $batch_size \
# 	SOLVER.BASE_LR $base_lr \
# 	SOLVER.MAX_ITER $num_train_steps \
# 	SOLVER.STEPS $LR_step_1,$LR_step_2 \