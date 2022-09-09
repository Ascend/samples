caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hpa/hpa.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hpa/hpa.prototxt"
model_name="deploy_vel"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/hpa_classification/test_image/"
data_name="test.jpeg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/hpa_classification/verify_image/"
verify_name="verify.jpeg"
project_name="human_protein_map_classification"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/human_protein_map_classification
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 main.py ../data"
model_atc="atc --model=./hpa.prototxt --weight=./hpa.caffemodel --framework=0 --output=${HOME}/models/${project_name}/${model_name}  --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data --output_type=FP32 --out_nodes="score:0""

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
