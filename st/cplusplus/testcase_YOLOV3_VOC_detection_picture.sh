tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_VOC_detection_picture/yolov3_tf.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_VOC_detection_picture/insert_op.cfg"
model_name="yolov3"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_VOC_detection_picture/test_image/"
data_name="boat.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_VOC_detection_picture/verify_image/"
verify_name="verify_boat.jpg"
project_name="cplusplus_YOLOV3_VOC_detection_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/2_object_detection/YOLOV3_VOC_detection_picture/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="input/input_data:1,416,416,3" --input_fp16_nodes="" --output_type=FP32 --input_format=NHWC --output_type=FP32
"

. ${script_path}/../../../../../common/testcase_common.sh

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

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main

