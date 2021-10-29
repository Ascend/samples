data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/venc/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/venc/verify_source/"
data_name="detection.mp4"
verify_name="out_video.h264"
project_name="cplusplus_venc"
version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/detection.mp4"

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

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

    run_picture
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}
    elif [ $? -eq ${verifyResError} ];then
	return ${verifyResError}
    fi

    return ${success}
}
main

