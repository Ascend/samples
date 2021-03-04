data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/200dk/SentimentAnalysis.zip"
project_name="SentimentAnalysis"
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

version=$1
declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {
    mkdir -p ${project_path}/data/
    if [[ ! -f "${project_path}/data/${project_name}.zip" ]] || [[ ! -d "${project_path}/data/${project_name}" ]];then
        wget -O ${project_path}/data/${project_name}.zip  ${data_source}  --no-check-certificate
        unzip -o ${project_path}/data/${project_name}.zip -d ${project_path}/data/
        mv ${project_path}/data/${project_name}/* ${project_path}/data/
        # cp -r ${project_path}/data/coarse-big-corpus ${project_path}/data/
        cp -r ${project_path}/data/snapshots/* ${project_path}/models/snapshots/
        cp -r ${project_path}/data/chinese_L-12_H-768_A-12/* ${project_path}/models/chinese_L-12_H-768_A-12/
        cp -r ${project_path}/data/jsoncpp ${project_path}/models/
    fi

    if [ $? -ne 0 ];then
        echo "download .zip failed, please check Network."
        return 1
    fi
    return 0
}

function setAtcEnv() {
    # 设置模型转换时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export PYTHONPATH=${install_path}/atc/python/site-packages/te:${install_path}/atc/python/site-packages/topi:$PYTHONPATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    fi

    return 0
}

function setBuildEnv() {
    # 设置代码编译时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}



function main() {
    downloadDataWithVerifySource
    
    setAtcEnv
    echo "begin model convert"
    cd ${project_path}/src/acl_demo
    ./model_convert.sh
    echo "finished model convert"
    
    setBuildEnv
    echo "begin compile acl/c++ code"
    ./build.sh
    echo "finished compile acl/c++ code"

    echo "finished npu inference"
    mkdir -p ../../output
    ./build/inference -m ../../models/snapshots/models.om -i ../../models/hotel.decode.txt -o ../../output/
    echo "finished npu inference"

    echo "output test result="
    cd ../../models
    python check_output.py
    
    ## GPU/CPU TEST
    # python main.py
    # python test.py
    
    echo "run success"
    return ${success}
}

main
