data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/jpege/"
data_name="dvpp_output.yuv"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/jpege/verify_source/"
verify_name="jpege_output.jpg"
project_name="cplusplus_jpege"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/jpege/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main"

. ${common_script_dir}/testcase_common.sh

function main() {

    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
