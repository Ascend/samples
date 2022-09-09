tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/painting/AIPainting_v2.pb"
model_name="AIPainting_v2"
presenter_server_name="display"
project_name="cplusplus_AI_painting"
conf_file_name="param.conf"
script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/AI_painting/scripts
project_path=${script_path}/..
common_script_dir=${script_path}
run_command="./${project_name}"
input_shape="objs:9;coarse_layout:1,256,256,17"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --output_type=FP32 --input_shape=${input_shape} --input_format=NHWC "

. ${common_script_dir}/../../../../common/testcase_common.sh

function main() {
    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_presenter
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    echo "run success"
}
main