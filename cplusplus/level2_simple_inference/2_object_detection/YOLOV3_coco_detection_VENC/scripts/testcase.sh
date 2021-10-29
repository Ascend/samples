caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
aipp_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_VENC/aipp_bgr.cfg"
model_name="yolov3"
data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_VENC/detection.mp4"

project_name="cplusplus_YOLOV3_coco_detection_VENC"

data_name="detection.mp4"

version=$1
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/detection.mp4"
model_atc=" atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} 
"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${script_path}/../../../../../common/testcase_common.sh

function main() {

    # 下载测试集和验证集
    wget -O ${project_path}/data/"detection.mp4"  ${data_source}  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test1.jpg failed, please check Network."
        return 1
    fi
    mkdir -p ${project_path}/verify_image/

    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi   

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}
    elif [ $? -eq ${verifyResError} ];then
	return ${verifyResError}
    fi

    return ${success}
}
main

