data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/resize/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/resize/verify_source/"
project_name="cplusplus_crop"
data_name="wood_rabbit_1024_1068_nv12.yuv"
verify_name="output.yuv"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/wood_rabbit_1024_1068_nv12.yuv 1024 1068 ./output/output.yuv 224 224"

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

