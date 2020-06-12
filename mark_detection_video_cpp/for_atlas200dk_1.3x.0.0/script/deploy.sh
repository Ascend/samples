#!/bin/bash
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
app_path="${script_path}/../src"

. ${script_path}/func_util.sh

check_param_configure()
{
    for i in `cat ${app_path}/param_configure.conf | awk -F'[ =]+' '{print $2}'`
    do
        if [[ ${i} = "" ]];then
            echo "please check your param_configure.conf to make sure that each parameter has a value"
            return 1
        fi
    done 

    #get and check format of remost_host ip
    check_remote_host
    if [ $? -ne 0 ];then
		return 1
    fi 

    #check format of data_source
    data_source=`cat ${app_path}/param_configure.conf | grep "data_source" | awk -F'[ =]+' '{print $2}'`
    if [[ ${data_source} != "Channel-1" && ${data_source} != "Channel-2" ]];then
        echo "ERROR: invalid camera channel name, please input Channel-1 or Channel-2."
        return 1
    fi

    #check format of presenter_view_app_name
    presenter_view_app_name=`cat ${app_path}/param_configure.conf | grep "presenter_view_app_name" | awk -F'[ =]+' '{print $2}' | grep "^[0-9a-zA-Z/]\+$"`
    if [[ ${presenter_view_app_name} = "" ]];then
        echo "ERROR: invalid presenter_view_app_name name,please input 0-9, a-z, A-Z, /"
        return 1
    fi
}


function build_common()
{
	echo "build common lib..."
    if [ ! -d "${HOME}/ascend_ddk" ];then
        mkdir $HOME/ascend_ddk
        if [[ $? -ne 0 ]];then
            echo "ERROR: Execute mkdir command failed, Please check your environment"
            return 1
        fi
    fi
    bash ${script_path}/build_ezdvpp.sh ${remote_host}
    if [ $? -ne 0 ];then
        echo "ERROR: Failed to deploy ezdvpp"
        return 1
    fi

    bash ${script_path}/build_presenteragent.sh ${remote_host}
    if [ $? -ne 0 ];then
        echo "ERROR: Failed to deploy presenteragent"
        return 1
    fi
    return 0
}


function main()
{
    echo "Modify param information in graph.config..."
    check_param_configure
    if [ $? -ne 0 ];then
        return 1
    fi

    build_common
    if [ $? -ne 0 ];then
        echo "ERROR: Failed to deploy common lib"
        return 1
    fi

    echo "echo Prepare app configuration..."
    parse_presenter_altasdk_ip ${remote_host}
    presenter_port=`grep presenter_server_port ${script_path}/../presenterserver/mask_detection/config/config.conf | awk -F '=' '{print $2}' | sed 's/[^0-9]//g'`
    if [ $? -ne 0 ];then
        echo "ERROR: get presenter server port failed, please check ${script_path}/../presenterserver/mask_detection/config/config.conf."
        return 1
    fi

    cp -r ${script_path}/graph_template.config ${app_path}/graph.config
    sed -i "s/\${template_data_source}/${data_source}/g" ${app_path}/graph.config
    sed -i "s/\${template_app_name}/${presenter_view_app_name}/g" ${app_path}/graph.config
    sed -i "s/\${template_presenter_ip}/${presenter_atlasdk_ip}/g" ${app_path}/graph.config
    sed -i "s/\${template_presenter_port}/${presenter_port}/g" ${app_path}/graph.config


    return 0
}
main
