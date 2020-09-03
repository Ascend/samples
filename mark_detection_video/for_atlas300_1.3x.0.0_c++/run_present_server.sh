#!/bin/bash
app_path="$( cd "$(dirname ${BASH_SOURCE})" ; pwd -P )"
app_name="maskdetection"

. ${app_path}/script/func_util.sh

function main()
{
    stop_pid=`ps -ef | grep "presenter_server\.py" | grep "${app_name}" | awk -F ' ' '{print $2}'`
    if [[ ${stop_pid}"X" != "X" ]];then
        echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${stop_pid}.\033[0m"
        kill -9 ${stop_pid}
    fi

    check_python3_lib
    if [ $? -ne 0 ];then
		return 1
    fi

    #get and check format of remost_host ip
    check_remote_host
    if [ $? -ne 0 ];then
	return 1
    fi 

    parse_presenter_altasdk_ip ${remote_host}

    parse_presenter_view_ip

    #1. After checking the ip, copy the ip value to config.config
    echo "Use ${presenter_atlasdk_ip} to connect to Atlas DK Developerment Board..."
    sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${presenter_atlasdk_ip}/g" ${app_path}/presenterserver/${app_name}/config/config.conf
    
    echo "Use ${presenter_view_ip} to show information in browser..."
    sed -i "s/web_server_ip=[0-9.]*/web_server_ip=${presenter_view_ip}/g" ${app_path}/presenterserver/${app_name}/config/config.conf
    
    echo "Finish to prepare ${app_name} presenter server ip configuration."
    
    python3 ${app_path}/presenterserver/presenter_server.py --app ${app_name} &

    return 0
}
main
