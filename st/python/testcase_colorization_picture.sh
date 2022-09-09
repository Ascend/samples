caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.prototxt"
model_name="colorization_yuv"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/colorization_picture-python/"
data_name="dog.png"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/colorization_picture-python/verify_image/"
verify_name="verify_dog.png"
project_name="python_colorization_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/colorization_picture
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 colorize.py ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape="data_l:1,1,224,224" --input_format=NCHW"

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
