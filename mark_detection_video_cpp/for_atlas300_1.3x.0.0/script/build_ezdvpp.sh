#!/bin/bash

script_path="$( cd "$(dirname ${BASH_SOURCE})" ; pwd -P )"
atlas_target=$1
ezdvpp_version="1.2.0"
DEVICE_LIB_PATH="${HOME}/ascend_ddk/device/lib"
AGENT_PATH="${HOME}/ascend_ddk"

function download_code()
{
    if [ -d ${AGENT_PATH}/ezdvpp ];then
        echo "EZdvpp code if found..."
        return 0
    else
        echo "Download ezdvpp code..."
        ezdvpp_download_url="https://gitee.com/Atlas200DK/sdk-ezdvpp/repository/archive/1.2.0?format=tar.gz"
        wget -O ${AGENT_PATH}/${ezdvpp_version}.ing ${ezdvpp_download_url} --no-check-certificate 1>/dev/null 2>&1
        if [[ $? -ne 0 ]];then
            echo "ERROR: download failed, please check ${ezdvpp_download_url} connection."
            return 1
        fi
    fi

    mv ${AGENT_PATH}/${ezdvpp_version}.ing ${AGENT_PATH}/${ezdvpp_version}
    tar -zxvf ${AGENT_PATH}/${ezdvpp_version} -C ${AGENT_PATH} 1>/dev/null
    if [[ $? -ne 0 ]];then
        echo "ERROR: uncompress ezdvpp tar.gz file failed, please check ${ezdvpp_download_url} connection."
        return 1
    fi
    mv ${AGENT_PATH}/sdk-ezdvpp ${AGENT_PATH}/ezdvpp
    rm -rf ${AGENT_PATH}/${ezdvpp_version}
    return 0
}

function build_ezdvpp()
{
    echo "Build ezdvpp..."
    if [ -e "${AGENT_PATH}/ezdvpp/out/libascend_ezdvpp.so" ];then
        echo "EZdvpp so is found.."
        return 0
    fi
    
    make clean mode=${atlas_target} -C ${AGENT_PATH}/ezdvpp 1>/dev/null 2>&1
    if [[ $? -ne 0 ]];then
        echo "ERROR: compile ezdvpp failed, please check the env."
        return 1
    fi

    make install mode=${atlas_target} -C ${AGENT_PATH}/ezdvpp 1>/dev/null 2>&1
    if [[ $? -ne 0 ]];then
        make clean mode=${atlas_target} -C ${AGENT_PATH}/ezdvpp 1>/dev/null 2>&1
        echo "ERROR: compile ezdvpp failed, please check the env."
        return 1
    fi
}

main()
{
    #download code
    download_code
    if [[ $? -ne 0 ]];then
        return 1
    fi
    build_ezdvpp
    if [[ $? -ne 0 ]];then
        return 1
    fi
    echo "Finish to Build ezdvpp."
		
    # 部署libascend_ezdvpp.so到/lib/device下
    echo "Start to deploy ezdvpp"
    mkdir -p ${script_path}/../lib/device
    if [[ $? -ne 0 ]];then
        echo "mkdir -p ${script_path}/../lib/device failed!"
        return 1
    fi
    cp ${DEVICE_LIB_PATH}/libascend_ezdvpp.so ${script_path}/../lib/device
    if [ $? -ne 0 ];then
        echo "cp ${DEVICE_LIB_PATH}/libascend_ezdvpp.so ${script_path}/../lib/device failed!"
        return 1
    fi
    echo "Finish to deploy ezdvpp"
    return 0
}
main


