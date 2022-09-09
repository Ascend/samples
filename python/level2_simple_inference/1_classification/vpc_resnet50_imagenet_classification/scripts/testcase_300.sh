caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/resnet50/resnet50.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/resnet50/resnet50.prototxt"
data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/"
model_name="resnet50_aipp"
project_name="python_vpc_resnet50_imagenet_classification"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function downloadData() {
    mkdir -p ${project_path}/data/

    if [ ! -f ${project_path}/data/"dog1_1024_683.jpg" ]; then
        wget -O ${project_path}/data/"dog1_1024_683.jpg"  ${data_source}"dog1_1024_683.jpg"  --no-check-certificate
        if [ $? -ne 0 ];then
            echo "download dog1_1024_683.jpg failed, please check Network."
            return 1
        fi
    fi

    if [ ! -f ${project_path}/data/"dog2_1024_683.jpg" ]; then
        wget -O ${project_path}/data/"dog2_1024_683.jpg"  ${data_source}"dog2_1024_683.jpg"  --no-check-certificate
        if [ $? -ne 0 ];then
            echo "download dog2_1024_683.jpg failed, please check Network."
            return 1
        fi
    fi

    return 0
}

function setEnv() {
    # 设置模型转换时需要的环境变量
    export PATH=/usr/local/python3.7.5/bin:$PATH
    if [ "$UID" = "0" ]; then
        source /usr/local/Ascend/ascend-toolkit/set_env.sh
    else
        source $HOME/Ascend/ascend-toolkit/set_env.sh
    fi

    return 0
}

function downloadOriginalModel() {
    mkdir -p ${project_path}/caffe_model/
    if [ ! -f ${project_path}/caffe_model/${caffe_prototxt##*/} ]; then
        wget -O ${project_path}/caffe_model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "install caffe_prototxt failed, please check Network."
            return 1
        fi
    fi

    if [ ! -f ${project_path}/caffe_model/${caffe_model##*/} ]; then
        wget -O ${project_path}/caffe_model/${caffe_model##*/} ${caffe_model} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "install caffe_model failed, please check Network."
            return 1
        fi
    fi

    return 0
}

function main() {

    # 生成模型输入数据集
    downloadData
    if [ $? -ne 0 ];then
        echo "ERROR: generate data failed"
        return ${inferenceError}
    fi

    # 下载原始模型文件
    downloadOriginalModel
    if [ $? -ne 0 ];then
        echo "ERROR: download original model failed"
        return ${inferenceError}
    fi

    setEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set environment var failed"
        return ${inferenceError}
    fi

    cd ${project_path}

    # 转模型
    atc --model=${project_path}/caffe_model/${caffe_prototxt##*/} --weight=${project_path}/caffe_model/${caffe_model##*/} --framework=0 --output=model/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/caffe_model/aipp.cfg
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
        return ${inferenceError}
    fi

    atc --singleop=${project_path}/op_models/op_list.json --soc_version=Ascend310 --output=${project_path}/op_models
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
        return ${inferenceError}
    fi

    # 运行程序
    python3 ./src/acl_sample.py | tee result.txt
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    # 验证结果
    results=$(cat result.txt)
    if [[ $results =~ "ERROR" || $results =~ "fail" || $results =~ "FAIL" || $results =~ "error" ]];then
        echo "run failed"
        return ${verifyResError}
    else
        echo "run success"
        return ${success}
    fi
}
main
