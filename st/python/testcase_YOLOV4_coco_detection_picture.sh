onnx_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_dynamic_bs.onnx"
model_name="yolov4_bs1"
project_name="python_YOLOV4_coco_detection_picture-python"
data_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV4_coco_detection_picture-python/"
verify_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV4_coco_detection_picture-python/"
verify_name="out_test.jpg"
data_name="test.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 yolov4.py"

. ${common_script_dir}/testcase_common.sh

function main() {
    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/model
    if [ ! -f "${project_path}/model/yolov4_bs1.om" ];then
        wget  https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOv4_onnx/yolov4_bs1.om
    fi


    run_picture_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi
    return ${success}
}
main


