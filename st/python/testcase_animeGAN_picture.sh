tf_model="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/animeGAN/AnimeGAN.pb"
model_name="AnimeGANv2"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/animeGAN/test_image/"
data_name="test.jpg"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/animeGAN/verify_image/"
verify_name="verify_test.jpg"
project_name="animeGAN_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/animeGAN_picture
common_script_dir=${project_path}/../../../../common
run_command="python3.6 AnimeGAN.py ../data/"
model_atc="atc --output_type=FP32 --input_shape="test:1,512,512,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=3 --save_original_model=false --model=${project_path}/model/${tf_model##*/} --precision_mode=allow_fp32_to_fp16"
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
