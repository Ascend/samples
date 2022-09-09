caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt"
model_name="face_detection"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_detection-python/insert_op.cfg"
presenter_server_name="face_detection"


data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_detection_rtsp/"
data_name="person.h264"
project_name="python_face_detection_rtsp"
conf_file_name="face_detection.conf"
run_command="python3.6 main.py"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/face_detection_rtsp/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,300,300" --input_format=NCHW --output_type=FP32  --save_original_model=false"

. ${common_script_dir}/testcase_common.sh

function main() {
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi 
    # 开启presenter server
    run_presenter_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
