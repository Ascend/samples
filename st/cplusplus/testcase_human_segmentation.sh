conf_file_name="human_segmentation.conf"

tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/human_segmentation/human_segmentation.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/human_segmentation/insert_op.cfg"

model_name="human_segmentation"
presenter_server_name="human_segmentation"
conf_file_name="human_segmentation.conf"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/human_segmentation/"
data_name="person.mp4"
project_name="cplusplus_human_segmentation_video"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/human_segmentation/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../common/
run_command="./${project_name} ${project_path}/data/${data_name}"
model_atc="atc --input_shape="input_rgb:1,512,512,3" --output=${HOME}/models/${project_name}/${model_name} --insert_op_conf=${project_path}/model/insert_op.cfg --framework=3 --model=${project_path}/model/${tf_model##*/} --soc_version=Ascend310  --input_format=NHWC
"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

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
