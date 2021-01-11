#!/bin/bash

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
exec_file=${project_path}/out/graph_run

declare -i success=0
declare -i runError=1
declare -i verifyResError=2

function envCheck() {
    export ASCEND_INSTALL_PATH=$HOME/Ascend/ascend-toolkit/latest
    if [ -d ${ASCEND_INSTALL_PATH}/fwkacllib ]; then
        echo "WARNING: graph_run can only run with fwkacllib, please check it installed correctly!"
    fi
}

function setEnv() {
    # set environment
    export PYTHONPATH=${ASCEND_INSTALL_PATH}/fwkacllib/python/site-packages:${ASCEND_INSTALL_PATH}/fwkacllib/python/site-packages/auto_tune.egg/auto_tune:${ASCEND_INSTALL_PATH}/fwkacllib/python/site-packages/schedule_search.egg:$PYTHONPATH
    export LD_LIBRARY_PATH=${ASCEND_INSTALL_PATH}/fwkacllib/lib64
    export ASCEND_OPP_PATH=${ASCEND_INSTALL_PATH}/opp
    export PATH=${ASCEND_INSTALL_PATH}/fwkacllib/ccec_compiler/bin:${ASCEND_INSTALL_PATH}/fwkacllib/bin:$PATH
}

function compile() {
    # compile 
    cd ${project_path}
    make
    if [ $? -ne 0 ]; then
        echo "ERROR: make failed."
        return ${runError}
    fi
}

function runCase() {
    cd ${project_path}/data
    python3 data_generate.py
    if [ $? -ne 0 ]; then
        echo "ERROR: generate binary file failed."
        return ${runError}
    fi

    ${exec_file}
    if [ $? -ne 0 ];then
        echo "ERROR: generate model failed."
        return ${runError}
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

    echo "run success"
    return ${success}
}

main
