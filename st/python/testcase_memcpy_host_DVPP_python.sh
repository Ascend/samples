script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level1_single_api/1_acl/2_memory_management/memcpy_host_DVPP_python
common_script_dir=${project_path}/../../../../../common/
run_command="python3 memcpy_host_dvpp.py --release_cycle 10 --number_of_cycles 1 --device_id 0 --memory_size 10485760 --memory_reuse"

. ${common_script_dir}/testcase_common.sh

function main() {
    setEnv
    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    
    return ${success}
}
main
