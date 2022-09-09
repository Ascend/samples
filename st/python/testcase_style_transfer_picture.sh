tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/style_transfer_picture/xingkong1.pb"
model_name="xingkong1_fp32_nchw_no_aipp"

data_source="https://obs-9be7.obs.myhuaweicloud.com/models/style_transfer_picture/data/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.myhuaweicloud.com/models/style_transfer_picture/verify/"
verify_name="verify_test.jpg"
verify_name2="verify2_test.jpg"
project_name="python_style_transfer"
run_command="python3.6 main.py ../data xingkong"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/6_other/style_transfer_picture
model_atc="atc --model=${project_path}/model/${tf_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310"
common_script_dir=${project_path}/../../../../common/

. ${common_script_dir}/testcase_common.sh

function main() {
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    downloadVerifySource2
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi 

    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    verify_ret=1
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out -name "*${tmp#*_}"`;do
            python3.6 ${common_script_dir}/verify_result.py ${test_file} ${outimage}
            if [ $? -eq 0 ];then
		verify_ret=0
            fi
        done
    done
    if [ ${verify_ret} -ne 0 ];then
	echo "ERROR: The result of reasoning is wrong!"
    fi

    return ${success}
}
main
