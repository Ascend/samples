tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoon/cartoonization.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/insert_op.cfg"
model_name="cartoonization"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/test_image/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/verify_image/"
verify_name="verify_test.jpg"
project_name="python_cartoonGAN_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/contrib/cartoonGAN_picture
common_script_dir=${project_path}/../../../common/
run_command="python3.6 cartoonization.py ../data"
model_atc="atc --output_type=FP32 --input_shape="train_real_A:1,256,256,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --framework=3 --save_original_model=false --model=${project_path}/model/${tf_model##*/} --precision_mode=allow_fp32_to_fp16"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi


    # 转模型
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
