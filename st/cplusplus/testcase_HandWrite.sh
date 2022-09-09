caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/handwrite/resnet.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/handwrite/resnet.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/handwrite/insert_op.cfg"
model_name="resnet"

presenter_server_name="hand_write"
conf_file_name="param.conf"
project_name="cplusplus_hand_write"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/HandWrite/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./${project_name}"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,112,112" --input_format=NCHW"

. ${common_script_dir}/testcase_common.sh

function main() {
    # 转模型
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
