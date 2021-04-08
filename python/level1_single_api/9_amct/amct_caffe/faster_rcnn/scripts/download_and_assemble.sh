#!/bin/bash
set -e
##############"Step1: Mkdir of 'caffe_master_patch' for custom layer"##############
echo "[INFO]Step1: Mkdir of 'caffe_master_patch' for custom layer"
SAMPLE_SRC_DIR=../src/

rm -rf caffe_master_patch
mkdir -p caffe_master_patch/src/caffe/layers
mkdir -p caffe_master_patch/include/caffe

rm -rf ${SAMPLE_SRC_DIR}/pre_model
mkdir -p ${SAMPLE_SRC_DIR}/pre_model

rm -rf ${SAMPLE_SRC_DIR}/python_tools
mkdir -p ${SAMPLE_SRC_DIR}/python_tools
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/datasets
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/nms
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/rpn
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/utils
mkdir -p ${SAMPLE_SRC_DIR}/python_tools/tools

rm -rf ${SAMPLE_SRC_DIR}/datasets
mkdir -p ${SAMPLE_SRC_DIR}/datasets
##############"Step2: Git clone py-faster-rcnn from github"##############
echo "[INFO]Step2: Git clone py-faster-rcnn from github"
#######download roi-pooling custom layer
FILE=faster_rcnn_caffe_master.zip
if [ ! -f $FILE ]; then
    echo "[INFO]Download faster_rcnn_caffe_master from github"
    github_dir=https://github.com/rbgirshick/caffe-fast-rcnn/archive/0dcd397b29507b8314e252e850518c5695efbb83.zip
    wget --no-check-certificate $github_dir -O $FILE
fi
if [[ ! -f $FILE ]]; then
    echo "[ERROR]Download py-faster-rcnn failed, please retry init or copy faster_rcnn_caffe_master.zip file to this dir"
    exit -1
fi
echo "[INFO]Unzipping faster_rcnn_caffe_master.zip..."
unzip $FILE

cp caffe-fast-rcnn-0dcd397b29507b8314e252e850518c5695efbb83/src/caffe/layers/roi_pooling_layer.cpp caffe_master_patch/src/caffe/layers
cp caffe-fast-rcnn-0dcd397b29507b8314e252e850518c5695efbb83/src/caffe/layers/roi_pooling_layer.cu caffe_master_patch/src/caffe/layers
cp caffe-fast-rcnn-0dcd397b29507b8314e252e850518c5695efbb83/include/caffe/fast_rcnn_layers.hpp caffe_master_patch/include/caffe

rm -rf caffe-fast-rcnn-0dcd397b29507b8314e252e850518c5695efbb83
########download faster rcnn python code
TARGET_PATH=py-faster-rcnn-master
rm -rf ${TARGET_PATH}
mkdir -p ${TARGET_PATH}

FILE=py-faster-rcnn-master.zip
if [[ ! -f $FILE ]]; then
    echo "[INFO]Download rbgirshick/py-faster-rcnn from github"
    github_dir=https://github.com/rbgirshick/py-faster-rcnn/archive/master.zip
    wget --no-check-certificate $github_dir -O $FILE
fi
if [[ ! -f $FILE ]]; then
    echo "[ERROR]Download py-faster-rcnn-master failed, please retry init or copy master.zip file to this dir"
    exit -1
fi
echo "[INFO]unzipping py-faster-rcnn-master..."
unzip $FILE

cp ${TARGET_PATH}/lib/fast_rcnn/__init__.py ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn
cp ${TARGET_PATH}/lib/fast_rcnn/bbox_transform.py ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn
cp ${TARGET_PATH}/lib/fast_rcnn/config.py ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn
cp ${TARGET_PATH}/lib/fast_rcnn/nms_wrapper.py ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn
cp ${TARGET_PATH}/lib/fast_rcnn/test.py ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn

cp ${TARGET_PATH}/lib/nms/__init__.py ${SAMPLE_SRC_DIR}/python_tools/nms
cp ${TARGET_PATH}/lib/nms/cpu_nms.pyx ${SAMPLE_SRC_DIR}/python_tools/nms
cp ${TARGET_PATH}/lib/nms/gpu_nms.hpp ${SAMPLE_SRC_DIR}/python_tools/nms
cp ${TARGET_PATH}/lib/nms/gpu_nms.pyx ${SAMPLE_SRC_DIR}/python_tools/nms
cp ${TARGET_PATH}/lib/nms/nms_kernel.cu ${SAMPLE_SRC_DIR}/python_tools/nms
cp ${TARGET_PATH}/lib/nms/py_cpu_nms.py ${SAMPLE_SRC_DIR}/python_tools/nms

cp ${TARGET_PATH}/lib/rpn/__init__.py ${SAMPLE_SRC_DIR}/python_tools/rpn
cp ${TARGET_PATH}/lib/rpn/generate_anchors.py ${SAMPLE_SRC_DIR}/python_tools/rpn
cp ${TARGET_PATH}/lib/rpn/proposal_layer.py ${SAMPLE_SRC_DIR}/python_tools/rpn

cp ${TARGET_PATH}/lib/utils/__init__.py ${SAMPLE_SRC_DIR}/python_tools/utils
cp ${TARGET_PATH}/lib/utils/bbox.pyx ${SAMPLE_SRC_DIR}/python_tools/utils
cp ${TARGET_PATH}/lib/utils/blob.py ${SAMPLE_SRC_DIR}/python_tools/utils
cp ${TARGET_PATH}/lib/utils/timer.py ${SAMPLE_SRC_DIR}/python_tools/utils

cp ${TARGET_PATH}/lib/Makefile ${SAMPLE_SRC_DIR}/python_tools
cp ${TARGET_PATH}/lib/setup.py ${SAMPLE_SRC_DIR}/python_tools

cp ${TARGET_PATH}/data/demo/* ${SAMPLE_SRC_DIR}/datasets

cp ${TARGET_PATH}/tools/demo.py ${SAMPLE_SRC_DIR}/python_tools/tools
cp ${TARGET_PATH}/lib/utils/__init__.py ${SAMPLE_SRC_DIR}/python_tools/tools
# Download VGG16 faster_rcnn model file from github
cp ${TARGET_PATH}/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test.pt ${SAMPLE_SRC_DIR}/pre_model
MODEL_FILE=faster_rcnn_models.tgz
if [[ ! -f $MODEL_FILE ]]; then
    echo "[INFO]Download Faster_RCNN pretrained models..."
    bash ${TARGET_PATH}/data/scripts/fetch_faster_rcnn_models.sh
    bash ${TARGET_PATH}/data/scripts/fetch_faster_rcnn_models.sh
    if [[ ! -f  ${TARGET_PATH}/data/faster_rcnn_models/VGG16_faster_rcnn_final.caffemodel ]]; then
        echo "[ERROR]Download Faster_RCNN pretrained models filed, please retry init or copy faster_rcnn_models.tgz file to this dir"
        exit -1
    fi
    cp ${TARGET_PATH}/data/faster_rcnn_models/VGG16_faster_rcnn_final.caffemodel ${SAMPLE_SRC_DIR}/pre_model
    cp ${TARGET_PATH}/data/$MODEL_FILE ./
else
    tar zxvf $MODEL_FILE
    cp faster_rcnn_models/VGG16_faster_rcnn_final.caffemodel ${SAMPLE_SRC_DIR}/pre_model
    rm -rf faster_rcnn_models
fi

if [[  $# -eq 1 && $1 == 'with_benchmark' ]]; then
    cp ${TARGET_PATH}/tools/test_net.py ${SAMPLE_SRC_DIR}/python_tools/tools

    cp ${TARGET_PATH}/lib/datasets/__init__.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/imdb.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/pascal_voc.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/factory.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/coco.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/voc_eval.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp ${TARGET_PATH}/lib/datasets/ds_utils.py ${SAMPLE_SRC_DIR}/python_tools/datasets
    cp -r ${TARGET_PATH}/lib/datasets/tools ${SAMPLE_SRC_DIR}/python_tools/datasets


    mkdir -p ${SAMPLE_SRC_DIR}/experiments/cfgs
    cp ${TARGET_PATH}/experiments/cfgs/faster_rcnn_alt_opt.yml ${SAMPLE_SRC_DIR}/experiments/cfgs
    # Download datasets
    DATASETS_VOC_2007=VOCtrainval_06-Nov-2007.tar
    UNZIP_FILE=VOCdevkit
    if [[ ! -f $DATASETS_VOC_2007 ]]; then
        echo "[INFO]Download VOC2007 datasets"
        wget http://host.robots.ox.ac.uk/pascal/VOC/voc2007/VOCtrainval_06-Nov-2007.tar
    fi
    if [[ ! -f $DATASETS_VOC_2007 ]]; then
        echo "[ERROR]Download VOC2007 datasets failed, please retry init or copy VOCtrainval_06-Nov-2007.tar file to this dir"
        exit -1
    fi
    tar xvf $DATASETS_VOC_2007 -o $UNZIP_FILE
    mv VOCdevkit ${SAMPLE_SRC_DIR}/datasets/
    cd ${SAMPLE_SRC_DIR}/datasets/ && ln -s VOCdevkit VOCdevkit2007 && cd -

    # Generate test images index
    TARGET_FILE=${SAMPLE_SRC_DIR}/datasets/VOCdevkit/VOC2007/ImageSets/Main/test.txt
    for i in `ls ${SAMPLE_SRC_DIR}/datasets/VOCdevkit/VOC2007/JPEGImages/`; do
        echo $(echo $i | sed 's/\.[^.]*$//') >> ${TARGET_FILE}
    done
    mkdir -p ${SAMPLE_SRC_DIR}/datasets/VOCdevkit/results/VOC2007/Main
fi

rm -rf ${TARGET_PATH}

