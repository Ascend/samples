data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vdecandvenc/"
data_name="person.mp4"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vdecandvenc/verify_source/"
verify_name="dvpp_venc.h264"
project_name="cplusplus_vdecandvenc"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/vdecandvenc/scripts
project_path=${script_path}/..
run_command="./main ${project_path}/data/${data_name}"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${script_path}/../../../../../common/testcase_common.sh

function main() {

    # 下载测试集
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
        return ${inferenceError}
    fi
}
main
