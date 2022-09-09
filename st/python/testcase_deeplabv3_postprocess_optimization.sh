mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/deeplabv3_quant.air"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/aipp.cfg"
model_name="deeplab_quant"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/deeplabv3/test_image/"
data_name="test.jpg"
project_name="python_deeplabv3_postprocess_optimization"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/3_segmentation/deeplabv3_postprocess_optimization
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 deeplab_dvpp_aipp_quant.py"
run_command_multi="python3.6 deeplab_multi.py"
model_atc="atc --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=1 --model=${project_path}/model/${mindspore_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/}"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    i=0
    while [ $i -le 6 ]
    do
      let i++
      cp ${project_path}/data/${data_name} ${project_path}/data/test${i}.jpg
    done

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

    run_command=${run_command_dvpp}
    
    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main
