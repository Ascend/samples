caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_picture/aipp_nv12.cfg"
model_name="yolov3"

data_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_freetype/test_image/"
verify_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_freetype/verify_image/"
project_name="cplusplus_YOLOV3_coco_detection_picture_with_freetype"
data_name="boat_512_384.jpg"
verify_name="out_boat_512_384.yuv"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture_with_freetype/scripts/
project_path=${script_path}/..

common_script_dir=${script_path}/../../../../../common/

run_command="./main ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/}"


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

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    mkdir -p ${project_path}/out/output
    cd ${project_path}/out
    ${run_command}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    return ${success}
}
main


