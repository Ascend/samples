#!/bin/bash
project_name="memcpy_host_device_cpp"
script_path_temp="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
script_path=${script_path_temp}/../../cplusplus/level1_single_api/1_acl/2_memory_management/memcpy_host_device_cpp/scripts
project_path=${script_path}/..
common_script_dir=${script_path_temp}/../../common/
. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证?    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return 1
    fi

    buildproject
    if [ $? -ne 0 ];then
        return 1
    fi

    cd ${project_path}/out
    running_command="./main --release_cycle 2 --number_of_cycles 2\
    --device_id 0 --memory_size 10485760"
    ${running_command}
    if [ $? -ne 0 ];then
      return 1
    fi
    return ${success}
}
main
