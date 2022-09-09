caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.prototxt"
model_name="colorization"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/colorization/test_image/"
data_name="dog.png"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/colorization/verify_image/"
verify_name="out_dog.png"
project_name="cplusplus_colorization"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/6_other/colorization/scripts/
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data"
model_atc="atc --input_shape="data_l:1,1,224,224" --weight=${project_path}/model/${caffe_model##*/} --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=0 --model=${project_path}/model/${caffe_prototxt##*/}"

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

    run_picture
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main

