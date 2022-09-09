mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_mindspore/resnet-90_1875.air"
model_name="resnet50"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_mindspore/"
data_name="airplane.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_mindspore/"
verify_name="airplane_verify.jpg"
project_name="resnet50_mindspore_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/resnet50_mindspore_picture
common_script_dir=${script_path}/../../../../../common
run_command="python3.6 classify.py ../data"
model_atc="atc --model=${project_path}/model/${mindspore_model##*/} --framework=1 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310"

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
