tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/frozen_graph.pb"
davinci_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/frozen_graph-inception-resnet-test1.om"

model_name="frozen_graph-inception-resnet-test1"
                                                                                                                                        
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/InceptionV2/test_image/"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/InceptionV2/verify_image/"
project_name="InceptionV2"

data_name="test1.jpg"
verify_name="verify_test1.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/inceptionv2_picture
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 classify.py ../data"
model_atc="atc --model=${project_path}/model/${tf_model##*/}  --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310  --input_shape="input:1,299,299,3" --input_format=NHWC"

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

    run_picture_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
