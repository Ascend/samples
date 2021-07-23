caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.prototxt"

model_name="colorization"
presenter_server_name="colorization"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization_video/"
data_name="black-white_video.mp4"
project_name="cplusplus_colorization_video"
conf_file_name="colorization.conf"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
run_command="./${project_name} ${project_path}/data/${data_name}"
model_atc="atc --input_shape="data_l:1,1,224,224" --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_format=NCHW"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


. ${script_path}/../../../../../common/testcase_common.sh

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
