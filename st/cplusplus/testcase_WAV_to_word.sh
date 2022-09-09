tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Wav2word/Wav2word.pb"
model_name="voice"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/WAV_to_word/"
data_name="nihao.wav"
data_name2="xinpian.wav"
project_name="cplusplus_WAV_to_word"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/5_nlp/WAV_to_word/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main"
model_atc="atc --model=${project_path}/model/${tf_model##*/}  --soc_version=Ascend310  --input_shape="the_input:1,1600,200,1" --input_format=NHWC  --output=${HOME}/models/${project_name}/voice  --framework=3"

. ${script_path}/../../../../../common/testcase_common.sh

function main() {
    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    downloadData2
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

    cd ${script_path}
    python3 preparedata.py
    mkdir -p ../out/output

    cd ${project_path}/out
    # 运行程序
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi  

    cd ${project_path}/scripts
    python3 postprocess.py > result.txt
    temp=`cat result.txt | grep "文本"|awk -F'[: ]+' '{print $2}'`
    if [[ $temp != "你好呀" ]];then
        echo "ERROR: run failed. please check your project"
        return ${verifyResError}
    fi 

    echo "run success"

    return ${success}

}
main
