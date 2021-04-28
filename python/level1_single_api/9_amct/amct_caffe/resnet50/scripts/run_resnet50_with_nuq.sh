#!/bin/bash
help(){
    echo "usage running on GPU mode : bash scripts/run_resnet50_with_nuq.sh -c your_caffe_dir -g gpu_id"
    echo "      running on CPU mode : bash scripts/run_resnet50_with_nuq.sh -c your_caffe_dir"
    exit
}
gpu_id=-1
while getopts :c:g: opt
do
    case $opt in
        c)
            caffe_dir=$OPTARG ;;
        g)
            gpu_id=$OPTARG ;;
        :)
            if [ $OPTARG = "c" ];then
                echo "caffe_dir must be specified with -c."
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
    echo "Running scripts/run_resnet50_with_nuq.sh on GPU mode..."
    python3 ./src/ResNet50_sample.py \
    --model model/ResNet-50-deploy.prototxt \
    --weights model/ResNet-50-model.caffemodel \
    --caffe_dir $caffe_dir \
    --cfg_define nuq_files/quant.cfg \
    --gpu $gpu_id
elif [ $gpu_id -eq -1 ];then
    echo "Running scripts/run_resnet50_with_nuq.sh on CPU mode..."
    python3 ./src/ResNet50_sample.py \
    --model model/ResNet-50-deploy.prototxt \
    --weights model/ResNet-50-model.caffemodel \
    --cfg_define nuq_files/quant.cfg \
    --caffe_dir $caffe_dir
fi