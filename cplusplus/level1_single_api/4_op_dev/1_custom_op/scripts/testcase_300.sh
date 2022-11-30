script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

version=$1

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function setCustomEnv() {
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export ASCEND_OPP_PATH=${install_path}/opp
        export ASCEND_TENSOR_COMPILER_INCLUDE=${install_path}/atc/include
        export TOOLCHAIN_DIR=${install_path}/toolkit/toolchain/hcc
        export SYSTEM_INFO=ubuntu_x86_64
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export ASCEND_OPP_PATH=${install_path}/opp
        export ASCEND_TENSOR_COMPILER_INCLUDE=${install_path}/atc/include
        export TOOLCHAIN_DIR=${install_path}/toolkit/toolchain/hcc
        export SYSTEM_INFO=ubuntu_x86_64
    fi

    return 0
}

function setCustomRun() {
    setCustomEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set custom environment failed"
        return ${inferenceError}
    fi
    custom_op_script=${project_path}/build.sh
    if [ ! -f $custom_op_script ];then
        echo "ERROR: custom op script is not exist"
        return ${inferenceError}
    else
        cd ${project_path}/1_custom_op
        chmod +x build.sh
        ./build.sh
        run_file=${project_path}/build_out/custom_opp_ubuntu_x86_64.run
        if [ ! -f $run_file ];then
            echo "ERROR: custom op run is not exist"
            return ${inferenceError}
        else
            cd ${project_path}/build_out/
            yes y | ./custom_opp_ubuntu_x86_64.run
        fi
    fi

    return 0
}

function setBuildEnv() {
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}

function main() {
    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # Set custom op environment
    setCustomRun
    if [ $? -ne 0 ];then
        echo "ERROR: set custom run failed"
        return ${inferenceError}
    fi

    # Generate test data
    cd ${project_path}/cpukernel/testcase/tf_test/unique
    python3 tf_unique.py
    if [ $? -ne 0 ];then
        echo "ERROR: unique testcase failed!"
        return ${verifyResError}
    fi

    return ${success}
}
main
