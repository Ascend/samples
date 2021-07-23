caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt"
aipp_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_camera/insert_op.cfg"
model_name="face_detection"


data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdecandvenc/"
data_name="person.mp4"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdecandvenc/verify_source/"
verify_name="dvpp_venc.h264"
project_name="cplusplus_vdecandvenc"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
run_command="./main ${project_path}/data/${data_name}"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/}  --output_type=FP32 --input_shape="data:1,3,300,300"  --input_format=NCHW   --save_original_model=false" 

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${script_path}/../../../../../common/testcase_common.sh

function main() {

    # 下载测试集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_h264
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}
    elif [ $? -eq ${verifyResError} ];then
        return ${verifyResError}
    fi

}
main
