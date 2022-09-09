tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/image_HDR_enhance/image_HDR_enhance.pb"
model_name="image_HDR_enhance"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/image_HDR_enhance/"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/image_HDR_enhance/"
project_name="image_HDR_enhance"
data_name="data0.png"
verify_name="verify0.png"
script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/image_HDR_enhance
project_path=${script_path}
common_script_dir=${script_path}/../../../common/

run_command="python3.6 main.py"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_shape="input:1,512,512,3" --input_format=NHWC --output_type=FP32"

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
