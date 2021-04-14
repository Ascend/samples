version=$1
model_data="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.data-00000-of-00001"
model_meta="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.meta"
model_index="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.index"
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
project_name="ModelZoo_Yuefu_TF"


declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function setRunEnv() {
    # 设置模型转换时需要的环境变量
    export install_path=/home/HwHiAiUser/Ascend/nnae/latest
    export LD_LIBRARY_PATH=/usr/local/Ascend/driver/lib64/common/:/usr/local/Ascend/driver/lib64/driver:/usr/local/Ascend/add-ons/:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=${install_path}/fwkacllib/lib64:/home/HwHiAiUser/Ascend/ascend-toolkit/20.1.rc1/arm64-linux/atc/lib64/:$LD_LIBRARY_PATH
    export PYTHONPATH=${install_path}/fwkacllib/python/site-packages:${install_path}/fwkacllib/python/site-packages/auto_tune.egg:${install_path}/fwkacllib/python/site-packages/schedule_search.egg:$PYTHONPATH
    export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
    export PYTHONPATH=/home/HwHiAiUser/Ascend/tfplugin/latest/tfplugin/python/site-packages:${install_path}/fwkacllib/python/site-packages/hccl:$PYTHONPATH
    export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/nnae/latest/opp

    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}
    if [[ $(find ${HOME}/models/${project_name} -name "poetry.data-00000-of-00001")"x" = "x" ]];then

        mkdir -p ${project_path}/models/

        cd ${project_path}/models/

        wget ${model_data} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "install model_data failed, please check Network."
            return 1
        fi
    fi
    if [[ $(find ${HOME}/models/${project_name} -name "poetry.meta")"x" = "x" ]];then

        mkdir -p ${project_path}/models/
        cd ${project_path}/models/

        wget ${model_meta} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "install model_data failed, please check Network."
            return 1
        fi
    fi
    if [[ $(find ${HOME}/models/${project_name} -name "poetry.index")"x" = "x" ]];then

        mkdir -p ${project_path}/models/
        cd ${project_path}/models/

        wget ${model_index} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "install model_data failed, please check Network."
            return 1
        fi
    fi

    ln -s ${HOME}/models/${project_name}/"poetry.data-00000-of-00001" ${project_path}/models/"poetry.data-00000-of-00001"
    if [ $? -ne 0 ];then
        echo "ERROR: failed to set model soft connection"
        return ${inferenceError}
    fi
    ln -s ${HOME}/models/${project_name}/"poetry.meta" ${project_path}/models/"poetry.meta"
    if [ $? -ne 0 ];then
        echo "ERROR: failed to set model soft connection"
        return ${inferenceError}
    fi
    ln -s ${HOME}/models/${project_name}/"poetry.index" ${project_path}/models/"poetry.index"
    if [ $? -ne 0 ];then
        echo "ERROR: failed to set model soft connection"
        return ${inferenceError}
    fi

    cd ${project_path}/src

    setRunEnv

    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    bash main_1p.sh &
    #bash ${project_path}/main_1p.sh
    if [ $? -ne 0 ];then
        echo "ERROR: The result of reasoning is wrong!"
        return ${verifyResError}
    fi

    sleep 120

    project_pid=`ps -ef | grep "python3.7" | grep "poetry_v2.py" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

    else 
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    echo "run success"

    return ${success}
}
main
