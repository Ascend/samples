tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/bert_text_classification.pb"
model_name="bert_text_classification"

project_name="python_bert_text_classification"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/"
data_name="sample.txt"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/5_nlp/bert_text_classification
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 bert_text_classification.py"
input_shape_txt="input_1:1,300;input_2:1,300"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape=${input_shape_txt} --input_format=ND --out_nodes=dense_1/Softmax:0 --soc_version=Ascend310 --op_select_implmode="high_precision" "

. ${common_script_dir}/testcase_common.sh

function run_bert_text_classification(){
    # 运行程序
    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    dataline=$(cat ${project_path}/out/prediction_label.txt)
    if [[ ${dataline} != "体育" ]];then
            echo "ERROR: Prediction results Error!"
            return ${verifyResError}
    fi
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

    run_bert_text_classification
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
