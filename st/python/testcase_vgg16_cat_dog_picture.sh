tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vgg16_cat_dog/vgg16_cat_dog.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vgg16_cat_dog/insert_op.cfg"
model_name="vgg16_cat_dog"
    
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vgg16_cat_dog/test_image/"
data_name="cat.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vgg16_cat_dog/verify_image/"
verify_name="cat.jpg"
project_name="vgg16_cat_dog"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/vgg16_cat_dog_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../../common/
run_command="python3.6 main.py "
model_atc="atc --output_type=FP32 --input_shape="input_1:1,224,224,3" --input_format=NHWC --model=${project_path}/model/${tf_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --output=${HOME}/models/${project_name}/${model_name} --framework=3 --soc_version=Ascend310"

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
