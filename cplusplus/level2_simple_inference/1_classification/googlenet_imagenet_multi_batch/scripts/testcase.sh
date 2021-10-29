caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.prototxt"
aipp_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_multi_batch/insert_op.cfg"
model_name="googlenet_multibatch"
data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_multi_batch/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_multi_batch/verify_image/"
project_name="cplusplus_googlenet_imagenet_multi_batch"
version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:2,3,224,224" --input_format=NCHW
"
declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"dog1_1024_683.bin"  ${data_source}"dog1_1024_683.bin"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test1.bin failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"dog2_1024_683.bin"  ${data_source}"dog2_1024_683.bin"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test2.bin failed, please check Network."
        return 1
    fi
    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/verify_test1.txt ${verify_source}"verify_test1.txt" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test1.txt failed, please check Network."
        return 1
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

