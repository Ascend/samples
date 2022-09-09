onnx_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_dynamic_bs.onnx"
model_name="yolov4_bs1"

project_name="YOLOV4_coco_detection_car_video"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV4_coco_detection_car_video/test_video/"
data_name="test_fast.mp4"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV4_coco_detection_car_video/verify_source/"
verify_name="test_fast.mp4"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_car_video
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 yolov4.py ../data/test_fast.mp4"
#model_atc="atc --model=${project_path}/model/${tensorflow_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape="Input:1,416,416,3""

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    # modelconvert
    cd ${project_path}/model
    if [ ! -f "${project_path}/model/yolov4_bs1.om" ];then
      wget  https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_bs1.om
    fi

    mkdir -p ${project_path}/out
    cd ${project_path}/src
    ${run_command}
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi
    if [ ! -s ${project_path}/out/${verify_name} ];then
        return ${inferenceError}
    fi
    echo "run success"
    return ${success}
}
main

