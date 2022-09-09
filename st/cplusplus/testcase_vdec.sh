data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vdec/"
data_name="vdec_h265_1frame_rabbit_1280x720.h265"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vdec/verify_image/"
verify_name="image1.yuv"
project_name="cplusplus_vdec"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/vdec/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_md5
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
