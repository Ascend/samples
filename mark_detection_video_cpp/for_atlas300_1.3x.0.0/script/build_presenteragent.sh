#!/bin/bash

script_path="$( cd "$(dirname ${BASH_SOURCE})" ; pwd -P )"
atlas_target=$1
presenteragent_version="1.2.0"
HOST_LIB_PATH="${HOME}/ascend_ddk/host/lib"
AGENT_PATH="${HOME}/ascend_ddk"

. ${script_path}/func_util.sh

function download_code()
{
    if [ -d ${AGENT_PATH}/presenteragent ];then
        echo "Presenteragent code is found..."
        return 0
    else
        echo "Download presenteragent code..."
        presenteragent_download_url="https://gitee.com/Atlas200DK/sdk-presenter/repository/archive/1.2.0?format=zip"
        wget -O ${AGENT_PATH}/presenteragent-${presenteragent_version}.ing ${presenteragent_download_url} --no-check-certificate 1>/dev/null 2>&1
    fi
    if [[ $? -ne 0 ]];then
        echo "ERROR: download failed, please check ${presenteragent_download_url} connection."
        return 1
    fi

    mv ${AGENT_PATH}/presenteragent-${presenteragent_version}.ing ${AGENT_PATH}/presenteragent-${presenteragent_version}.zip

    unzip ${AGENT_PATH}/presenteragent-${presenteragent_version}.zip -d ${AGENT_PATH} 1>/dev/null
    if [[ $? -ne 0 ]];then
        echo "ERROR: uncompress presenteragent tar.gz file failed, please check ${presenteragent_download_url} connection."
        return 1
    fi
	
	mkdir -p ${AGENT_PATH}/presenteragent;rm -rf ${AGENT_PATH}/presenteragent/*
    cp -rf  ${AGENT_PATH}/sdk-presenter/presenteragent/* ${AGENT_PATH}/presenteragent/

    rm -rf ${AGENT_PATH}/presenteragent-${presenteragent_version}.zip
    rm -rf ${AGENT_PATH}/sdk-presenter
    return 0

}

function build_presenteragent()
{
    echo "Build presenteragent..."
    if [ -e "${AGENT_PATH}/presenteragent/out/libpresenteragent.so" ];then
        echo "Presenteragent so is found.."
        return 0
    fi
    
    make clean mode=${atlas_target} -C ${AGENT_PATH}/presenteragent 1>/dev/null 2>&1
    if [[ $? -ne 0 ]];then
        echo "ERROR: compile presenteragent failed, please check the env."
        return 1
    fi
    
    make install mode=${atlas_target} -C ${AGENT_PATH}/presenteragent 1>/dev/null 2>&1
    if [[ $? -ne 0 ]];then
        make clean mode=${atlas_target} -C ${AGENT_PATH}/presenteragent 1>/dev/null 2>&1
        echo "ERROR: compile presenteragent failed, please check the env."
        return 1
    fi
}

function check_presenteragent_proto_version()
{
    pb_h_file=${AGENT_PATH}/presenteragent/proto/presenter_message.pb.h
    proto_file=${AGENT_PATH}/presenteragent/proto/presenter_message.proto
    check_proto_version $pb_h_file $proto_file
    if [ $? -eq 1 ];then
        echo "ERROR: regenerate presenteragent proto code failed"
        return 1
    fi

    return 0
}

main()
{
    download_code
    if [[ $? -ne 0 ]];then
        return 1
    fi

    check_presenteragent_proto_version
    if [[ $? -ne 0 ]];then
        return 1
    fi

    build_presenteragent
    if [[ $? -ne 0 ]];then
        return 1
    fi
    echo "Finish to Build presenteragent."

    echo "Start to deploy presenteragent"

    mkdir -p ${script_path}/../lib/host
    if [[ $? -ne 0 ]];then
        echo "mkdir -p ${script_path}/../lib/host failed!"
        return 1
    fi
    cp ${HOST_LIB_PATH}/libpresenteragent.so ${script_path}/../lib/host
    if [ $? -ne 0 ];then
        echo "cp ${HOST_LIB_PATH}/libpresenteragent.so ${script_path}/../lib/host failed!"
        return 1
    fi

    echo "Finish to deploy presenteragent"

    return 0
}
main
