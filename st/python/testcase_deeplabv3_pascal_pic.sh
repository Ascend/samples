tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com:443/003_Atc_Models/nkxiaolei/DeepLapV3_Plus/deeplabv3_plus.pb"
model_name="deeplabv3_plus"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/deeplabv3/"
data_name="dog.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/deeplabv3/"
verify_name="out_dog.jpg"
project_name="deeplabv3_pascal_pic"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/3_segmentation/deeplabv3_pascal_pic/scripts
project_path=${script_path_temp}/../../python/level2_simple_inference/3_segmentation/deeplabv3_pascal_pic
common_script_dir=${script_path_temp}/../../common/
run_command="python3.6 deeplabv3.py ../data"

#output_type_txt="SemanticPredictions:0:FP32"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_shape="ImageTensor:1,513,513,3" --output_type="SemanticPredictions:0:FP32" --out_nodes="SemanticPredictions:0" "
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
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
