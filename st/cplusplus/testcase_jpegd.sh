data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/jpegd/"
data_name="dog1_1024_683.jpg"
project_name="cplusplus_jpegd"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/jpegd/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/dog1_1024_683.jpg"

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
    cd ${project_path}/out
    ${run_command}
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
