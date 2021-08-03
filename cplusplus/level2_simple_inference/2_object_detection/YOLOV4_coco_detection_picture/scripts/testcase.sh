tf_model="https://nkxiaolei88.obs.cn-north-1.myhuaweicloud.com/ATC%20Model/YoloV4/yolov4_no_postprocess.pb"
data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/"
data_name="dog1_1024_683.jpg"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/verify_image/"
verify_name="out_dog1_1024_683.jpg"
model_name="yolov4"
project_name="cplusplus_yolov4_coco_detection_picture"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../model/yolov4.om ../data/dog1_1024_683.jpg"
model_atc="atc --input_shape="Input:1,416,416,3" --output=${HOME}/models/${project_name}/${model_name} --insert_op_conf=${project_path}/model/insert_op.cfg --framework=3 --model=${project_path}/model/${tf_model##*/} --soc_version=Ascend310"

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

