tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/yolo3_resnet18.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/insert_op.cfg"
model_name="yolo3_resnet18_yuv"
presenter_server_name="mask_detection_video"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/mask_detection_video/"
data_name="mask.h264"
project_name="YOLOV3_mask_detection_video"
conf_file_name="mask_detection.conf"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video/scripts
project_path=${script_path}/..
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 main.py ../data/mask.h264"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --output=${HOME}/models/${project_name}/${model_name}  --output_type=FP32 --input_shape="images:1,352,640,3" --input_format=NHWC --framework=3 --soc_version=Ascend310"

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

    run_presenter_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
