#!/bin/bash

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
input1_path=${project_path}/run/out/test_data/data/input_0.bin
input2_path=${project_path}/run/out/test_data/data/input_1.bin
output_path=${project_path}/run/out/result_files/output_0.bin

declare -i success=0
declare -i runError=1
declare -i verifyResError=2

function envCheck() {
    export ASCEND_INSTALL_PATH=$HOME/Ascend/ascend-toolkit/latest
    if [ -d ${ASCEND_INSTALL_PATH}/compiler ]; then
        echo "WARNING: op online compile can only run with compiler, please check it installed correctly!"
    fi
}

function setEnv() {
    # set environment
    export PYTHONPATH=${ASCEND_INSTALL_PATH}/compiler/python/site-packages:${ASCEND_INSTALL_PATH}/compiler/python/site-packages/auto_tune.egg/auto_tune:${ASCEND_INSTALL_PATH}/compiler/python/site-packages/schedule_search.egg:$PYTHONPATH
    export LD_LIBRARY_PATH=${ASCEND_INSTALL_PATH}/compiler/lib64
    export ASCEND_OPP_PATH=${ASCEND_INSTALL_PATH}/opp
    export PATH=${ASCEND_INSTALL_PATH}/compiler/ccec_compiler/bin:${ASCEND_INSTALL_PATH}/compiler/bin:$PATH
}

function compile() {
    # compile 
    # Create a directory to hold the compiled files
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

    # Set the environment variables that your code needs to compile
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
}

function runCase() {
    # Generate test data
    cd ${project_path}/run/out/test_data/data
    python3 generate_data.py
    if [ $? -ne 0 ];then
        echo "ERROR: generate input data failed!"
        return ${verifyResError}
    fi

    cd ${project_path}/run/out
    # Run the program
    ./execute_add_op
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
}

function main() {
    envCheck
    setEnv
    compile
    if [ $? -ne 0 ]; then
        echo "ERROR: compile failed, please check your project."
        return ${runError}
    fi

    runCase
    if [ $? -ne 0 ]; then
        echo "ERROR: run case failed, please check your project."
        return ${runError}
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
