script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
input1_path=${project_path}/run/out/test_data/data/input_0.bin
input2_path=${project_path}/run/out/test_data/data/input_1.bin
output_path=${project_path}/run/out/result_files/output_0.bin

version=$1

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function setCustomEnv() {
    export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
    export ASCEND_OPP_PATH=${install_path}/opp
    export ASCEND_TENSOR_COMPLIER_INCLUDE=${install_path}/compiler/include
    export TOOLCHAIN_DIR=${install_path}/toolkit/toolchain/hcc
    export SYSTEM_INFO=ubuntu_x86_64

    return 0
}

function setCustomRun() {
    setCustomEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set custom environment failed"
        return ${inferenceError}
    fi
    custom_op_script=${project_path}/../../1_custom_op/build.sh
    if [ ! -f $custom_op_script ];then
        echo "ERROR: custom op script is not exist"
        return ${inferenceError}
    else
        cd ${project_path}/../../1_custom_op
        chmod +x build.sh
        ./build.sh
        run_file=${project_path}/../../1_custom_op/build_out/custom_opp_ubuntu_x86_64.run
        if [ ! -f $run_file ];then
            echo "ERROR: custom op run is not exist"
            return ${inferenceError}
        else
            cd ${project_path}/../../1_custom_op/build_out/
            yes y | ./custom_opp_ubuntu_x86_64.run
        fi
    fi

    return 0
}

function setAtcEnv() {
    export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
    export PATH=/usr/local/python3.7.5/bin:${install_path}/compiler/ccec_compiler/bin:${install_path}/compiler/bin:$PATH
    export ASCEND_OPP_PATH=${install_path}/opp
    export PYTHONPATH=${install_path}/compiler/python/site-packages:${install_path}/compiler/python/site-packages/auto_tune.egg/auto_tune:${install_path}/compiler/python/site-packages/schedule_search.egg:$PYTHONPATH
    export LD_LIBRARY_PATH=${install_path}/compiler/lib64:${LD_LIBRARY_PATH}

    return 0
}

function setBuildEnv() {
    export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux
    export NPU_HOST_LIB=${DDK_PATH}/runtime/lib64/stub

    return 0
}

function main() {
    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]] || [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        echo "WARNING: ${version} is not support custom aicpu ops"
        return ${success}
    fi

    # Set custom op environment
    setCustomRun
    if [ $? -ne 0 ];then
        echo "ERROR: set custom run failed"
        return ${inferenceError}
    fi

    # Set atc environment
    setAtcEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set atc environment failed"
        return ${inferenceError}
    fi

    # Generate model from json
    cd ${project_path}/run/out
    atc --singleop=test_data/config/reshape_op.json --soc_version=Ascend310 --output=op_models
    if [ $? -ne 0 ];then
        echo "ERROR: convert model failed"
        return ${inferenceError}
    fi

    # Generate test data
    cd ${project_path}/run/out/test_data/data
    python3 generate_data.py
    if [ $? -ne 0 ];then
        echo "ERROR: generate input data failed!"
        return ${verifyResError}
    fi

    # Create a directory to hold the compiled files
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

    # Set the environment variables that your code needs to compile
    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    # Generate Makefile
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "ERROR: cmake failed. please check your project"
        return ${inferenceError}
    fi

    make
    if [ $? -ne 0 ];then
        echo "ERROR: make failed. please check your project"
        return ${inferenceError}
    fi

    cd ${project_path}/run/out

    # Reconfigure the environment variables required for the program to run
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/runtime/lib64:$LD_LIBRARY_PATH

    # Run the program
    ./execute_custom_reshape_op
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    # Call the python script to determine if the results of this project reasoning are correct
    python3 ${script_path}/verify_result.py ${input1_path} ${input2_path} ${output_path}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. the result of reasoning is wrong!"
        return ${inferenceError}
    fi

    echo "run success"

    return ${success}
}
main
