caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/dege_detection/rcf_bsds.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/dege_detection/rcf.prototxt"
model_name="rcf"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/rcf_edge_detection/"
data_name="ori.jpg"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/rcf_edge_detection/"
verify_name="out_ori.jpg"
project_name="edge_detection_picture"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/contrib/edge_detection_picture
common_script_dir=${project_path}/../../../common/
run_command="python3.6 main.py ../data"

model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_fp16_nodes=data  --output_type=FP32 --input_format=NCHW"
  
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
