data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/venc/"
data_name="dvpp_vpc_1920x1080_nv12.yuv"
verify_name="out_video.h264"
project_name="cplusplus_venc"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/venc/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/dvpp_vpc_1920x1080_nv12.yuv"

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

    mkdir -p ${project_path}/out/output
    # 运行程序
    cd ${project_path}/out
    ${run_command}
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi
    if [ ! -s ${project_path}/out/output/${verify_name} ];then
        return ${inferenceError}
    fi
    return ${success}
}
main
