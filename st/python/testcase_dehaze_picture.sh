tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/SingleImageDehaze/output_graph.pb"
model_name="deploy_vel"

project_name="SingleImageDehaze"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/SingleImageDehaze/test_image/"
data_name="10992_04_0.8209.png"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/SingleImageDehaze/verify_image/"
verify_name="out_10992_04_0.8209.png"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/contrib/dehaze_picture
common_script_dir=${project_path}/../../../common/
run_command="python3.6 main.py ../data"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --input_shape="t_image_input_to_DHGAN_generator:1,512,512,3" --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_fp16_nodes="t_image_input_to_DHGAN_generator" --output_type= FP32"

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
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
