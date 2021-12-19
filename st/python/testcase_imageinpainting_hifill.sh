json_model="https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json"
model_name="0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/imageinpainting_hifill/data/"
data_name="test.jpg"
mask_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/imageinpainting_hifill/mask/"
mask_name="test.jpg"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/imageinpainting_hifill/verify_image/"
verify_name="verify_test.jpg"
project_name="imageinpainting_hifill"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/imageinpainting_hifill
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 main.py"
model_atc="atc --singleop=./matmul_27648.json --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310"

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

    #modelconvert
    cd ${project_path}/model
    if [ ! -f "${project_path}/model/hifill.om" ];then
        wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om --no-check-certificate
	if [ $? -ne 0 ];then
            return 1
        fi
    fi
    cd ${project_path}/model/
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        if [[ ${json_model}"x" != "x" ]];then
            wget -O ${project_path}/model/${json_model##*/} ${json_model} --no-check-certificate
            if [ $? -ne 0 ];then
                echo "wget json_model failed, please check Network."
                return ${inferenceError}
            fi
        fi
        ${model_atc}
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi
    fi

    if [[ $(find ${project_path}/model -name ${model_name}".om")"x" != "x" ]];then
        rm ${project_path}/model/${model_name}".om"
    fi
    cp   ${HOME}/models/${project_name}/${model_name}/${model_name}".om" ${project_path}/model/
    if [ $? -ne 0 ];then
        echo "ERROR: failed to set model soft connection"
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
