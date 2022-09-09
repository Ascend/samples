tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/mask_detection/mask_detection.pb"
model_name="mask_detection"

project_name="python_YOLOV3_mask_detection_picture"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_mask_detection_picture-python/test_image/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_mask_detection_picture-python/verify_image/"
verify_name="verify_test.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 mask_detect.py"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape="images:1,352,640,3" --input_format=NHWC"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture_python
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
