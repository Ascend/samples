caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_multi_thread_VENC/aipp_bgr.cfg"
model_name="yolov3"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_multi_thread_VENC/test_image/"
data_name="video1.mp4"
project_name="YOLOV3_coco_detection_4_thread"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_4_thread/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../../common/
run_command="./main"
model_atc="atc  --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --insert_op_conf=${project_path}/model/${aipp_cfg##*/}"

. ${script_path}/../../../../../../common/testcase_common.sh

function main() {

    # 下载测试集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cp ${project_path}/data/video1.mp4 ${project_path}/data/video2.mp4 
    cp ${project_path}/data/video1.mp4 ${project_path}/data/video3.mp4 
    cp ${project_path}/data/video1.mp4 ${project_path}/data/video4.mp4 

    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    mkdir -p ${project_path}/out/output
    cd ${project_path}/out
    ${run_command}
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi
    return ${success}
}
main
