#!/bin/bash
app_scripts_path="$( cd "$(dirname ${BASH_SOURCE})" ; pwd -P )"
app_path=$app_scripts_path/../
app_name="hand_write"

presenter_connect_ip=""
presenter_view_ip=""

function check_ip_addr()
{
    echo "check ip $1"

    ip_addr=$1
    ip_addr=$(echo "$ip_addr" | grep "^[0-9]\{1,3\}\.\([0-9]\{1,3\}\.\)\{2\}[0-9]\{1,3\}$")
    if [ -z "$ip_addr" ]
    then
	    echo "ip_addr $ip_addr invalid"
        return 1
    fi

    for num in `echo ${ip_addr} | sed "s/./ /g"`
    do
        if [ $num -gt 255 ] || [ $num -lt 0 ]
        then
            return 1
        fi
   done
   return 0
}

function check_python3_lib()
{
    echo "Check python3 libs ......"

    tornado_obj=$(cat ${app_path}/presenterserver/requirements | grep tornado | awk -F'[ =]+' '{print $2}')
    if [ $? -ne 0 ];then
        echo "ERROR: please check your env."
        return 1
    elif [ 5.1.0 = ${tornado_obj} ];then
		tornado_obj=5.1
    fi


    protobuf_obj=$(cat ${app_path}/presenterserver/requirements | grep protobuf | awk -F'[ =]+' '{print $2}')
    if [ $? -ne 0 ];then
        echo "ERROR: please check your env."
        return 1
    fi
    
    if tornado=$(python3 -c "import tornado;print(tornado.version)" 2>/dev/null);then
		if [ ${tornado} != ${tornado_obj} ];then
	    	pip3 install tornado==${tornado_obj} --user 2>/dev/null
     		if [ $? -ne 0 ];then
        		echo "ERROR: install tornado failed, please check your env."
        		return 1
        	fi
		fi
    else
		pip3 install tornado==${tornado_obj} --user 2>/dev/null
		if [ $? -ne 0 ];then
	    	echo "ERROR: install tornado failed, please check your env."
            return 1
        fi
    fi 

    if protobuf=$(python3 -c "import google.protobuf;print(google.protobuf.__version__)" 2>/dev/null);then
		if [ ${protobuf} != ${protobuf_obj} ];then
	    	pip3 install protobuf==${protobuf_obj} --user 2>/dev/null
     	    if [ $? -ne 0 ];then
        		echo "ERROR: install protobuf failed, please check your env."
        		return 1
            fi
		fi
    else
		pip3 install protobuf==${protobuf_obj} --user 2>/dev/null
		if [ $? -ne 0 ];then
	    	echo "ERROR: install protobuf failed, please check your env."
            return 1
        fi
    fi 
    
    numpy=$(python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null)
    if [ $? -ne 0 ];then
		pip3 install numpy --user 2>/dev/null
		if [ $? -ne 0 ];then
	    	echo "ERROR: install numpy failed, please check your env."
            return 1
        fi
    fi
    echo "python3 libs have benn prepared."
}

function check_ip()
{
    #check format of remost_host ip
    presenter_connect_ip=$(cat ${app_path}/scripts/param.conf | grep "presenter_server_ip" | awk -F'[ =]+' '{print $2}')
	presenter_connect_ip=$(echo $presenter_connect_ip | sed -e 's/\r//' | sed -e 's/\n//' | sed -e 's/ //')
    if [[ "$presenter_connect_ip" = "" ]];then
        echo "please check your param.conf to make sure that each parameter has a value"
        return 1
    fi
    check_ip_addr $presenter_connect_ip
    if [ $? -ne 0 ];then
        echo "ERROR: invalid presenter_connect_ip ip, please check your settings in configuration file"
        return 1
    fi

    presenter_view_ip=$(cat ${app_path}/scripts/param.conf | grep "presenter_view_ip" | awk -F'[ =]+' '{print $2}')
    presenter_view_ip=$(echo $presenter_view_ip | sed -e 's/\r//' | sed -e 's/\n//' | sed -e 's/ //')
       if [[ "$presenter_view_ip" = "" ]];then
           echo "please check your param.conf to make sure that each parameter has a value"
           return 1
       fi
    check_ip_addr $presenter_view_ip
    if [ $? -ne 0 ];then
       echo "ERROR: invalid presenter_view_ip ip, please check your settings in configuration file"
       return 1
    fi
}

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

    check_ip

    #1.检查完毕ip之后，将ip的数值复制到config.config 
    echo "Use ${presenter_connect_ip} to connect to Atlas DK Developerment Board..."
    sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${presenter_connect_ip}/g" ${app_path}/presenterserver/${app_name}/config/config.conf
    
    echo "Use ${presenter_view_ip} to show information in browser..."
    sed -i "s/web_server_ip=[0-9.]*/web_server_ip=${presenter_view_ip}/g" ${app_path}/presenterserver/${app_name}/config/config.conf
    
    echo "Finish to prepare ${app_name} presenter server ip configuration."
    
    python3 ${app_path}/presenterserver/presenter_server.py --app ${app_name} &

    return 0
}
main
