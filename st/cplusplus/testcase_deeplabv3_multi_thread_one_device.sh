mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/deeplabv3_quant.air"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/aipp.cfg"
model_name="deeplab_quant"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/cplusplus_test_image/"
data_name="data.tar.gz"
project_name="deeplabv3_multi_thread_one_device"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/deeplabv3_multi_thread_one_device/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../../common/
run_command="./main"
model_atc="atc --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=1 --model=${project_path}/model/${mindspore_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/}"

. ${script_path}/../../../../../../common/testcase_common.sh

function main() {

    # 下载测试集
    if [ ! -f "${project_path}/data.tar.gz" ];then
        downloadDataWithVerifySource
        mv ${project_path}/data/data.tar.gz ${project_path}/
    fi
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    tar -zxvf ${project_path}/data.tar.gz -C ${project_path}

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
