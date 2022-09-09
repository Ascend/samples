caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.prototxt"

model_name="colorization"
presenter_server_name="colorization"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/colorization_video/"
data_name="black-white_video.mp4"
project_name="cplusplus_colorization_video"
conf_file_name="colorization.conf"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/6_other/colorization_video/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./${project_name} ${project_path}/data/${data_name}"
model_atc="atc --input_shape="data_l:1,1,224,224" --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_format=NCHW"

. ${common_script_dir}/testcase_common.sh

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
