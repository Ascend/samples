tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/portrait.pb"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/insert_op.cfg"
model_name="portrait"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/Portrait/"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/Portrait/"
project_name="portrait_person_pic"

data_name="ori.jpg"
data_name2="background.jpg"
verify_name="mask_ori.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/portrait_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 main.py"
input_txt="Inputs/x_input:1,224,224,3"
model_atc="atc --model=${project_path}/model/${tf_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --output=${HOME}/models/${project_name}/${model_name}  --output_type=FP32 --input_shape=${input_txt} --framework=3 --soc_version=Ascend310"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    downloadData2
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

    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out/mask -name "*${tmp#*_}"`;do
            python3.6 ${common_script_dir}/verify_result.py ${test_file} ${outimage}
            if [ $? -ne 0 ];then
                echo "ERROR: The result of reasoning is wrong!"
                return ${verifyResError}
            fi
        done
    done

    echo "run success"


    return ${success}
}

main
