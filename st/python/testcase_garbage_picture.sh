mindspore_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com:443/003_Atc_Models/AE/ATC%20Model/garbage/mobilenetv2.air"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/insert_op_yuv.cfg"
model_name="garbage_yuv"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/"
data_name="bottle.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/"
verify_name="out_bottle.jpg"
project_name="garbage_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/garbage_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 classify_test.py ../data"
model_atc="atc --model=${project_path}/model/${mindspore_model##*/} --framework=1 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,224,224" --input_format=NCHW"

. ${common_script_dir}/testcase_common.sh
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

    run_picture_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
