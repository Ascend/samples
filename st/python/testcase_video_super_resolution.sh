tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb"
model_name="EDVR_180_320"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/"
data_name="low_resolution.mp4"
project_name="python_video_super_resolution"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/video_super_resolution
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 video_super_resolution.py"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --input_shape="L_input:1,5,180,320,3" --input_format=ND --soc_version=Ascend310  --output_type=FP32"
. ${common_script_dir}/testcase_common.sh

function main() {

    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    mkdir -p ${project_path}/out

    cd ${project_path}/src
    ${run_command}
    
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
