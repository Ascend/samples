mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air"
model_name="mnist"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/lenet_mindspore/test_image/"
data_name="test1.png"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/lenet_mindspore/verify_file/"
verify_name="verify_test1.txt"
project_name="lenet_mindspore_picture"


script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/lenet_mindspore_picture/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="python3.6 src/classify.py ./data/"
model_atc="atc --model=${project_path}/model/${mindspore_model##*/} --framework=1 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310"
. ${common_script_dir}/testcase_common.sh

function run_txt_python()
{
    mkdir -p ${project_path}/outputs
    cd ${project_path}
    ${run_command}
    
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/outputs" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/outputs -name "*${tmp#*_}"`;do
            python3.6 ${script_path}/verify_result.py ${test_file} ${outimage}
            if [ $? -ne 0 ];then
                echo "ERROR: The result of reasoning is wrong!"
                return ${verifyResError}
            fi   
        done
    done

    echo "run success"
    return ${success}
}

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

    run_txt_python
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
