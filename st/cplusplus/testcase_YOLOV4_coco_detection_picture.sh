tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/nkxiaolei/YoloV4/yolov4_no_postprocess.pb"
model_name="yolov4"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/nkxiaolei/YoloV4/insert_op.cfg"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/"
data_name="dog1_1024_683.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/verify_image/"
verify_name="out_dog1_1024_683.jpg"
project_name="cplusplus_yolov4_coco_detection_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../model/yolov4.om ../data/dog1_1024_683.jpg"
model_atc="atc --input_shape="Input:1,416,416,3" --output=${HOME}/models/${project_name}/${model_name} --insert_op_conf=${project_path}/model/insert_op.cfg --framework=3 --model=${project_path}/model/${tf_model##*/} --soc_version=Ascend310"

. ${script_path}/../../../../../common/testcase_common.sh

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

