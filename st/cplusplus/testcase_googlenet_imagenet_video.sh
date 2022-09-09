caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/googlenet_imagenet_video/insert_op.cfg"
model_name="googlenet"
presenter_server_name="display"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/googlenet_imagenet_video/"
data_name="cat.mp4"
project_name="cplusplus_googlenet_imagenet_video"
conf_file_name="param.conf"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./${project_name} ${project_path}/data/${data_name}"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,224,224" --input_format=NCHW"


declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


. ${common_script_dir}/testcase_common.sh
#. ${script_path}/../../../../../common/testcase_common.sh

function main() {

    # 下载测试集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

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