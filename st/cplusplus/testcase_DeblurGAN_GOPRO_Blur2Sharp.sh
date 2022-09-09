tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeblurGAN/DeblurrGAN-pad-01051648.pb"
model_name="blurtosharp_pad_1280_720"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/DeblurGAN_GOPRO_Blur2Sharp/test_image/"
data_name="data.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/DeblurGAN_GOPRO_Blur2Sharp/verify_image/"
verify_name="verify_data.jpg"
project_name="cplusplus_DeblurGAN_GOPRO_Blur2Sharp"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data"
model_atc="atc --input_shape="blur:1,720,1280,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=3 --model=${project_path}/model/${tf_model##*/}"

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
