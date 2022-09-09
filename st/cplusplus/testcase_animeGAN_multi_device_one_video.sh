tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/animeGAN/AnimeGAN_256_256.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/animeGAN/aipp_256_256.cfg"
model_name="AnimeGANv2_256"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/animeGAN/test_video/"
data_name="test_video.mp4"
project_name="animeGAN_video"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/animeGAN_multi_device_one_video
common_script_dir=${project_path}/../../../../../common
run_command="./main 256 256"
model_atc="atc --output_type=FP32 --input_shape="test:1,256,256,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=3 --save_original_model=false --model=${project_path}/model/${tf_model##*/} --precision_mode=allow_fp32_to_fp16 --insert_op_conf=${project_path}/model/${aipp_cfg##*/}"
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

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/out
    ${run_command}
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
