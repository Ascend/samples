data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/crop/test_image/"
data_name="wood_rabbit_1024_1068_nv12.yuv"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/crop/verify_source/"
verify_name="output.yuv"
project_name="cplusplus_crop"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/0_data_process/crop/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main ../data/wood_rabbit_1024_1068_nv12.yuv 1024 1068 ./output/output.yuv 350 280 224 224"

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

    run_md5
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
