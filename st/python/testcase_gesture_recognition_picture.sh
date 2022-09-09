caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesture_recognition/resnet18_gesture.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesture_recognition/resnet18_gesture.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesture_recognition/insert_op.cfg"
model_name="gesture_yuv"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/gesture_recognition/test_image/"
data_name="test1.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/gesture_recognition/verify_image/"
project_name="gesture_recognition_picture"
verify_name="verify_test1.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/gesture_recognition_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 main.py ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,224,224" --input_format=NCHW"
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
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
