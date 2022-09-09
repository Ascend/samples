tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/ARshadowGAN/model.pb"
model_name="model"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/arshadow/data/"
verify_source='https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/arshadow/verify_image/'
project_name="ARshadow"
verify_name="result_0.jpg"
data_name1="test_A.png.bin"
data_name2="test_B.png.bin"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/ar_shadowgan/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./main ../data"
input_shape="placeholder/input_image:1,256,256,3;placeholder/input_mask:1,256,256,1"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape=${input_shape} --output_type=FP32 --input_format=NHWC --precision_mode=allow_fp32_to_fp16"

. ${common_script_dir}/testcase_common.sh
function main() {
    # 下载测试集
    data_name=${data_name1}
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    data_name=${data_name2}
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
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main
