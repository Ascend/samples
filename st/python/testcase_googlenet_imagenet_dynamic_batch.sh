caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/classification/googlenet.prototxt"
aipp_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch-python/insert_op.cfg"
model_name="googlenet_dynamic_batch"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch-python/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_multi_batch-python/verify_image/"
project_name="python_googlenet_imagenet_dynamic_batch"
data_name="test1.jpg"
verify_name="verify_test1.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level2_simple_inference/1_classification/googlenet_imagenet_dynamic_batch
project_path=${script_path}
common_script_dir=${script_path}/../../../../common/
run_command="python3.6 classify.py ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:-1,3,224,224" --input_format=NCHW --dynamic_batch_size="1,2,4,8""
. ${common_script_dir}/testcase_common.sh

function main() {
    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    if [ ! -f "${project_path}/data/test2.jpg" ];then
        wget -O ${project_path}/data/test2.jpg https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch-python/test_image/test2.jpg --no-check-certificate
    fi

    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture_python
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main
