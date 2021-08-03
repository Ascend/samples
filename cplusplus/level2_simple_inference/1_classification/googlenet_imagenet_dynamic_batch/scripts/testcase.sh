




caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.prototxt"
aipp_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch/insert_op.cfg"
model_name="googlenet_dynamicbatch"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch/test_image/"
data_name="test1.bin" 
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch/verify_image/"
verify_name="verify_test1.txt"
project_name="cplusplus_googlenet_imagenet_dynamic_batch"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/

run_command="./main "
model_atc=" atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:-1,3,224,224" --dynamic_batch_size="1,2" --output_type=FP32 --input_format=NCHW"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

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