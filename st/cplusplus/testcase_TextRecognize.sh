crnn_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/crnn_static/crnn_static.pb"
dbnet_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/dbnet/dbnet.pb"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/TextRecognize/"
data_name="data.mp4"
dbnet_model_name="dbnet"
crnn_model_name="crnn_static"
presenter_server_name="text_recognize"
project_name="TextRecongnize"
conf_file_name="TextRecongnize.conf"
script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/TextRecognize/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./${project_name}"
model_atc1="atc --model=${project_path}/model/${dbnet_model##*/}  --framework=3 --output=${HOME}/models/${project_name}/${dbnet_model_name} --soc_version=Ascend310  --output_type=FP32 --input_shape="input_images:1,736,1312,3" --input_format=NHWC"
model_atc2="atc --model=${project_path}/model/${crnn_model##*/}  --framework=3 --output=${HOME}/models/${project_name}/${crnn_model_name} --soc_version=Ascend310  --input_shape="new_input:1,32,100,3" --input_format=NHWC"

. ${common_script_dir}/testcase_common.sh
function main() {
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    if [ ! -f "${script_path}/../data/char_dict_en.json" ];then
        wget -O ${script_path}/../data/char_dict_en.json  ${data_source}char_dict_en.json --no-check-certificate
    fi

    if [ ! -f "${script_path}/../data/ord_map_en.json" ];then
        wget -O ${script_path}/../data/ord_map_en.json  ${data_source}ord_map_en.json --no-check-certificate
    fi

    # 转模型
    tf_model=${crnn_model}
    model_atc=${model_atc2}
    model_name=${crnn_model_name}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    tf_model=${dbnet_model}
    model_atc=${model_atc1}
    model_name=${dbnet_model_name}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_presenter
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main