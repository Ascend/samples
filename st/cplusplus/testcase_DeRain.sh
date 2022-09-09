tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeRain/frozen_graph_noDWT_V2.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeRain/insert_op.cfg"
model_name="DeRain"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeRain/test_image/"
data_name="001_in.png"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeRain/verify_image/"
verify_name="verify_001_in.png"

project_name="DeRain"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/6_other/DeRain/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="degradated_image:1,256,256,1""

. ${script_path}/../../../../../common/testcase_common.sh

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

main
