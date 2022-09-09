onnx_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_dynamic_bs.onnx"
model_name="yolov4_bs1"
project_name="YOLOV4_coco_detection_car_picture"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV4_coco_detection_car_picture/test_image/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV4_coco_detection_car_picture/verify_image/"
verify_name="verify_test.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_car_picture
common_script_dir=${project_path}/../../../../common/

run_command="python3.6 yolov4.py"
#out_nodes_param="Conv_434:0;Conv_418:0;Conv_402:0"
#model_atc="atc --model=${project_path}/model/${onnx_model##*/} --framework=5 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_format=NCHW --input_shape="input:1,3,608,608" --out_nodes=${out_nodes_param}"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    #modelconvert
    cd ${project_path}/model
    if [ ! -f "${project_path}/model/yolov4_bs1.om" ];then
        wget  https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_bs1.om --no-check-certificate
        if [ $? -ne 0 ];then
            return ${inferenceError}
        fi
    fi

    run_picture_python
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main


