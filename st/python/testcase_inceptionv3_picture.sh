onnx_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/inceptionv3.onnx"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/aipp_inceptionv3_pth.config"
model_name="InceptionV3"
    
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/googlenet_imagenet_picture/test_image/"
data_name="test1.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/verigy_data/"
verify_name="verify_test.jpg"
project_name="python_inceptionv3_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/inceptionv3_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../../common/
run_command="python3.6 classify.py ../data"
model_atc="atc --model=${project_path}/model/${onnx_model##*/} --framework=5 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="actual_input_1:1,3,300,300" --input_format=NCHW" 
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
