caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/super_resolution/SRCNN/SRCNN.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/super_resolution/SRCNN/SRCNN.prototxt"
model_name="SRCNN_768_768"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/super_resolution/test_image/"
data_name="butterfly_GT.bmp"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/super_resolution/verify_image/"
verify_name="butterfly_GT_SRCNN.png"
project_name="cplusplus_Super_Resolution"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/super_resolution/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./main ../data "
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_format=NCHW --input_shape="data: 1, 1, 768, 768" --output_type=FP32"

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

