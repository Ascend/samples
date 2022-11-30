mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/deeplabv3_origin.air"
aipp_cfg_dvpp="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/aipp.cfg"
model_name="deeplab_origin"
model_name_dvpp="deeplab_aipp"

data_source="https://share123.obs.cn-north-4.myhuaweicloud.com/AclBot/issue/"
data_name="test.jpg"
project_name="python_deeplabv3"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/3_segmentation/deeplabv3
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 deeplab_origin.py"
run_command_dvpp="python3.6 deeplab_dvpp_aipp.py"
model_atc="atc --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=1 --model=${project_path}/model/${mindspore_model##*/}"
model_atc_dvpp="atc --output=${HOME}/models/${project_name}/${model_name_dvpp} --soc_version=Ascend310 --framework=1 --model=${project_path}/model/${mindspore_model##*/} --insert_op_conf=${aipp_cfg_dvpp##*/}"

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

    model_name=${model_name_dvpp}
    aipp_cfg=${aipp_cfg_dvpp}
    run_command=${run_command_dvpp}
    model_atc=${model_atc_dvpp}
    
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
