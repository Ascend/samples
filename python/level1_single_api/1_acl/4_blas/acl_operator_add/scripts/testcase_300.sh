script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function setEnv() {
    # 设置环境变量
    export PATH=/usr/local/python3.7.5/bin:$PATH
    if [ "$UID" = "0" ]; then
        source /usr/local/Ascend/ascend-toolkit/set_env.sh
    else
        source $HOME/Ascend/ascend-toolkit/set_env.sh
    fi

    return 0
}

function main() {
    # 设置环境变量
    setEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set atc environment failed"
        return ${inferenceError}
    fi

    # 转模型
    cd ${project_path}/test_data
    atc --singleop=config/add_op.json --soc_version=Ascend310 --output=op_models
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
        return ${inferenceError}
    fi

    python3 ../src/acl_execute_add.py | tee result.txt
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
