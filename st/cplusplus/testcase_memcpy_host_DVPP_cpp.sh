script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level1_single_api/1_acl/2_memory_management/memcpy_host_DVPP_cpp/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../../common/
run_command="./main --release_cycle 10 --number_of_cycles 1 --device_id 0 --memory_size 10485760 --memory_reuse 0"

. ${common_script_dir}/testcase_common.sh

function main() {
    setEnv
    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/out
    ${run_command}
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
