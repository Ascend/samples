#!/bin/bash
help(){
    echo "usage running on GPU mode : bash scripts/run_resnet50_auto_nuq.sh -c your_caffe_dir -d data_set_dir -g gpu_id"
    echo "      running on CPU mode : bash scripts/run_resnet50_auto_nuq.sh -c your_caffe_dir -d data_set_dir"
    exit
}
gpu_id=-1
while getopts :c:d:g: opt
do
    case $opt in
        c)
            caffe_dir=$OPTARG ;;
        d)
            dataset=$OPTARG ;;
        g)
            gpu_id=$OPTARG ;;
        :)
            if [ $OPTARG = "c" ];then
                echo "caffe_dir must be specified with -c."
                help
            elif [ $OPTARG = "d" ];then
                echo "data_set dir must be specified with -d"
                help
            elif [ $OPTARG = "g" ];then
                echo "gpu_id must be specified with -g when using GPU mode"
                help
            fi ;;
        ?)
            echo "unknown args $OPTARG"
            help ;;
    esac
done
if [ $# -eq 0 ];then
    help
fi


if [ $gpu_id -ne -1 ];then
    echo "Running scripts/run_resnet50_auto_nuq.sh on GPU mode..."
    python3 ./src/auto_nuq_resnet50_sample.py \
    --model model/ResNet-50-deploy.prototxt \
    --weights model/ResNet-50-model.caffemodel \
    --cfg_define ./src/nuq_files/quant.cfg \
    --dataset $dataset \
    --caffe_dir $caffe_dir \
    --gpu $gpu_id
elif [ $gpu_id -eq -1 ];then
    echo "Running scripts/run_resnet50_auto_nuq.sh on CPU mode..."
    python3 ./src/auto_nuq_resnet50_sample.py \
    --model model/ResNet-50-deploy.prototxt \
    --weights model/ResNet-50-model.caffemodel \
    --cfg_define ./src/nuq_files/quant.cfg \
    --dataset $dataset \
    --caffe_dir $caffe_dir
fi