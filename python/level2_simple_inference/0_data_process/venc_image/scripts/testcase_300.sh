data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function downloadData() {
    mkdir -p ${project_path}/data/

    if [ ! -f ${project_path}/data/"dvpp_venc_128x128_nv12.yuv" ]; then
        wget -O ${project_path}/data/"dvpp_venc_128x128_nv12.yuv"  ${data_source}"dvpp_venc_128x128_nv12.yuv"  --no-check-certificate
        if [ $? -ne 0 ];then
            echo "download dvpp_venc_128x128_nv12.yuv failed, please check Network."
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

function main() {

    # 生成模型输入数据集
    downloadData
    if [ $? -ne 0 ];then
        echo "ERROR: generate data failed"
        return ${inferenceError}
    fi

    setEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set atc environment failed"
        return ${inferenceError}
    fi

    cd ${project_path}

    # 运行程序
    python3 ./src/acl_venc.py | tee result.txt
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
