caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt"
model_name="resnet50"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/"
data_name2="dog1_1024_683.bin" 
project_name="resnet50_firstapp"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/1_classification/resnet50_firstapp/script
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common

run_command="./main "
model_atc=" atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${project_path}/model/${model_name} --soc_version=Ascend310"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集

    downloadData2
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
#    elif [ $? -eq ${verifyResError} ];then
#        return ${verifyResError}
    fi

    return ${success}
}
main
