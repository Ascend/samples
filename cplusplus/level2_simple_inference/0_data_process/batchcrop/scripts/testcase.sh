data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/"
project_name="cplusplus_batchcrop"
data_name="dvpp_vpc_1920x1080_nv12.yuv"
version=$1
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

common_script_dir=${script_path}/../../../../../common/
run_command="./main "


declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"dvpp_vpc_1920x1080_nv12.yuv"  ${data_source}"dvpp_vpc_1920x1080_nv12.yuv"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download dvpp_vpc_1920x1080_nv12.yuv failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}

    fi

    return ${success}
}
main

