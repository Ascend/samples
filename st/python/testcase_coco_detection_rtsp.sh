caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
model_name="yolov3_yuv"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/aipp_nv12.cfg"
presenter_server_name="coco_detection"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_detection_rtsp/"
data_name="person.h264"
project_name="python_coco_detection_rtsp"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/coco_detection_rtsp/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="python3.6 main.py ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=/home/HwHiAiUser/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --save_original_model=false "

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

    cd ${project_path}/src
    ${run_command} 
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main
