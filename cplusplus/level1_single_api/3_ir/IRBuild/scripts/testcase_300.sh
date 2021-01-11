#!/bin/bash

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
exec_file=${project_path}/out/ir_build

declare -i success=0
declare -i runError=1
declare -i verifyResError=2

function envCheck() {
    export ASCEND_INSTALL_PATH=$HOME/Ascend/ascend-toolkit/latest
    if [ ! -d ${ASCEND_INSTALL_PATH}/atc ]; then
        echo "WARNING: ge_ir can only run with atc or fwkacllib, please check one of them installed correctly!"
    fi
}

# set environment
function setEnv() { 
    if [ -d ${ASCEND_INSTALL_PATH}/atc ]; then
        export PYTHONPATH=${ASCEND_INSTALL_PATH}/atc/python/site-packages:${ASCEND_INSTALL_PATH}/atc/python/site-packages/auto_tune.egg/auto_tune:${ASCEND_INSTALL_PATH}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${ASCEND_INSTALL_PATH}/atc/lib64
    else
	echo "can not find atc, please check one of them installed correctly!"
	return runError
    fi
    export ASCEND_OPP_PATH=${ASCEND_INSTALL_PATH}/opp
    export PATH=/usr/local/python3.7.5/bin:$PATH
    return 0
}

# compile
function compile() {
    cd ${project_path}
    make ir_build
}

# run case
function runCase() {
    cd ${project_path}/data
    python3 data_generate.py
    if [ $? -ne 0 ]; then
        echo "ERROR: generate binary file failed."
        return ${runError}
    fi

    if [ -f ${exec_file} ]; then
        ${exec_file} Ascend310 gen
    else
	echo "ERROR: can not find exec file."
	return ${runError}
    fi
    if [ $? -ne 0 ]; then
        echo "ERROR: generate model failed."
        return ${runError}
    fi 
}

function main() {
    envCheck
    setEnv
    if [ $? -ne 0 ]; then
        echo "ERROR: set environment failed, please check your project"
        return ${runError}
    fi
    compile
    if [ $? -ne 0 ]; then
        echo "ERROR: compile failed, please check your project"
        return ${runError}
    fi
    runCase
    if [ $? -ne 0 ]; then
        echo "ERROR: run case failed, please check your project"
        return ${runError}
    fi

    echo "run success"
    return ${success}
}

main
