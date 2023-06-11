#!/bin/bash
project_name="memcpy_host_device_asycn_python"
script_path_temp="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
project_path=${script_path_temp}/../../python/level1_single_api/1_acl/2_memory_management/memcpy_host_device_async_python
common_script_dir=${project_path}/../../../../../common/
running_command="python3.7 memcpy_host_device_async_python.py  --release_cycle 1 --release_cycle 10 --number_of_cycles 1 --device_id 0 --memory_size 10485760"
. ${common_script_dir}/testcase_common.sh

function main() {


    setEnv
    cd ${project_path}/src
    ${running_command}
    if [ $? -ne 0 ];then
      return 1
    fi
    return ${success}
}
main
