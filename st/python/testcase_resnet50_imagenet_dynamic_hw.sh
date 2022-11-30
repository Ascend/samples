model_path="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/tf_resnet50.om"
tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/resnet50_tensorflow_1.7.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/aipp_resnet50.aippconfig"
model_name="tf_resnet50"

data_source="https://obs-9be7.obs.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/test_image/"
data_name="dog1_1024_683.jpg"
project_name="resnet50_imagenet_dynamic_hw-python"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_hw
common_script_dir=${project_path}/../../../../common/
run_command="python3 acl_net.py"
#model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="Placeholder:1,-1,-1,3" --dynamic_image_size='112,112;224,224;448,448'"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    
    # 转模型
    cd ${project_path}/model
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/tf_resnet50.om
    
    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    
    return ${success}
}
main
