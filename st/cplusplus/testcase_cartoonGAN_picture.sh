tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoon/cartoonization.pb"
model_name="cartoonization"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/test_image/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/cplus/"
verify_name="verify_test.jpg"
project_name="cplusplus_contrib_cartoonGAN_picture"


script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/cartoonGAN_picture/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./main ../data "
model_atc="atc --output_type=FP32 --input_shape="train_real_A:1,256,256,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=3 --save_original_model=false --model=${project_path}/model/${tf_model##*/} --precision_mode=allow_fp32_to_fp16
"
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
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}
    elif [ $? -eq ${verifyResError} ];then
        return ${verifyResError}
    fi

    return ${success}
}
main

