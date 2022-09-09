json_model="https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json"
pb1="https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.pb"
model_name1="0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648"
model_name2="hifill"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/imageinpainting_hifill/data/"
data_name="test.jpg"
mask_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/imageinpainting_hifill/mask/"
mask_name="test.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/imageinpainting_hifill/verify_image/"
verify_name="verify_test.jpg"
project_name="imageinpainting_hifill"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/imageinpainting_hifill
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 main.py"
input_txt="img:1,512,512,3;mask:1,512,512,1"
model_atc1="atc --singleop=./matmul_27648.json --output=${HOME}/models/${project_name}/${model_name1} --soc_version=Ascend310"
model_atc2="atc --output_type=FP32 --input_shape=${input_txt} --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name2} --soc_version=Ascend310 --framework=3 --save_original_model=false --model=${project_path}/model/${pb1##*/}"


. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    if [ ! -f "${project_path}/mask/${mask_name}" ];then
        wget -O ${project_path}/mask/${mask_name}  ${mask_source}${mask_name}  --no-check-certificate
        if [ $? -ne 0 ];then
            echo "download test.png failed, please check Network."
            return 1
        fi
    fi

    tf_model=""
    json_model=${json_model}
    model_name=${model_name1}
    model_atc=${model_atc1}
    modelconvert
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
        return ${inferenceError}
    fi
    mv ${HOME}/models/${project_name}/${model_name1}/* ${project_path}/model
    tf_model=${pb1}
    model_name=${model_name2}
    model_atc=${model_atc2}
    modelconvert
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
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
