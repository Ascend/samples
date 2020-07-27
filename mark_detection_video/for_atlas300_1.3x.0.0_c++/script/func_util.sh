#!/bin/bash 

script_path="$( cd "$(dirname ${BASH_SOURCE})" ; pwd -P )"
remote_port="22118"
DDK_BIN="$DDK_HOME/uihost/bin"
tools_path="${script_path}/.."
PROTOC="$DDK_HOME/bin/x86_64-linux-gcc5.4/protoc"

function check_python3_lib()
{
    echo "Install python3 libs: pip3 install -r ${tools_path}/presenterserver/requirements..."

    tornado_obj=`cat ${tools_path}/presenterserver/requirements | grep tornado | awk -F'[ =]+' '{print $2}'`
    if [ $? -ne 0 ];then
        echo "ERROR: please check your env."
        return 1
    elif [ 5.1.0 = ${tornado_obj} ];then
		tornado_obj=5.1
    fi


    protobuf_obj=`cat ${tools_path}/presenterserver/requirements | grep protobuf | awk -F'[ =]+' '{print $2}'`
    if [ $? -ne 0 ];then
        echo "ERROR: please check your env."
        return 1
    fi
        
    numpy_obj=`cat ${tools_path}/presenterserver/requirements | grep numpy | awk -F'[ =]+' '{print $2}'`
    if [ $? -ne 0 ];then
        echo "ERROR: please check your env."
        return 1
    fi
    
    if tornado=`python3 -c "import tornado;print(tornado.version)" 2>/dev/null`;then
		if [ ${tornado} != ${tornado_obj} ];then
	    	pip3 install tornado==${tornado_obj} -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com 2>/dev/null
     		if [ $? -ne 0 ];then
        		echo "ERROR: install tornado failed, please check your env."
        		return 1
        	fi
		fi
    else
		pip3 install tornado==${tornado_obj} -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com 2>/dev/null
		if [ $? -ne 0 ];then
	    	echo "ERROR: install tornado failed, please check your env."
            return 1
        fi
    fi 

    if protobuf=`python3 -c "import google.protobuf;print(google.protobuf.__version__)" 2>/dev/null`;then
		if [ ${protobuf} != ${protobuf_obj} ];then
	    	pip3 install protobuf==${protobuf_obj} -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com 2>/dev/null
     	    if [ $? -ne 0 ];then
        		echo "ERROR: install protobuf failed, please check your env."
        		return 1
            fi
		fi
    else
		pip3 install protobuf==${protobuf_obj} -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com 2>/dev/null
		if [ $? -ne 0 ];then
	    	echo "ERROR: install protobuf failed, please check your env."
            return 1
        fi
    fi 
    
    numpy_version=`python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null`
	if [ ! ${numpy_version} ];then
	    pip3 install numpy -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com 2>/dev/null
     	if [ $? -ne 0 ];then
        	echo "ERROR: install numpy failed, please check your env."
        	return 1
        fi
    fi 
    

    echo "python3 libs have benn prepared."
}

# ************************check remote file****************************************
# Description:  upload a file
# $1: remote file(relative ~/xxxxx)
# ******************************************************************************
function check_remote_file()
{
    filePath=$1
    if [ ! -n ${filePath} ];then
        return 2
    fi
    [[ $DDK_HOME = "" ]] && (echo "ERROR: invalid DDK_bin path, please make sure your \$DDK_HOME path is right.";return 2) 
    ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --hostcmd "wc -l ${filePath}"`
    if [[ $? -ne 0 ]];then
        return 1
    fi

    return 0
}


# ************************uplooad file****************************************
# Description:  upload a file
# $1: local file(absolute)
# $2: remote file path
# ******************************************************************************
function upload_file()
{
    local_file=$1
    remote_path=$2

    file_name=`basename ${local_file}`
    remote_file="${remote_path}/${file_name}"

    #check remote path
    check_remote_file ${remote_file}

    #check whether overwrite remote file
    if [[ $? -eq 0 ]];then
        if [[ ${is_overwrite} == "false" ]];then
            echo "${remote_file} already exists, skip to upload it."
            return 0
        else
            ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --hostcmd "rm ${remote_file}"`
            if [[ $? -ne 0 ]];then
                echo "ERROR: delete ${remote_host}:${remote_file} failed, please make sure your remote_host ip is right."
                return 1
            fi
        fi
	elif [[ $? -eq 2 ]];then
		return 1
	fi
    	

    ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --hostcmd "mkdir -p ${remote_path}"`
    if [[ $? -ne 0 ]];then
        echo "ERROR: mkdir ${remote_host}:${remote_path} failed, please make sure your remote_host ip is right."
        return 1
    fi

    #copy to remote path
    ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --sync ${local_file} ${remote_path}`
    if [[ $? -ne 0 ]];then
        echo "ERROR: sync ${local_file} to ${remote_host}:${remote_path} failed, please make sure your remote_host ip is right."
        return 1
    fi
    return 0
}


# ************************uplooad tar.gz file****************************************
# Description:  upload a file
# $1: local file(absolute)
# $2: remote path
# $3: is_uncompress(true/false, default:true)
# ******************************************************************************
function upload_tar_file()
{
    local_file=$1
    remote_path=$2

    file_name=`basename ${local_file}`
    remote_file="${remote_path}/${file_name}"

    upload_file ${local_file} ${remote_path}
    if [[ $? -ne 0 ]];then
        return 1
    fi

    #uncompress tar.gz file
    if [[ ${is_uncompress}"X" != "falseX" ]];then
        ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --hostcmd "tar -xvf ${remote_file} -C ${remote_path}/"`
        if [[ $? -ne 0 ]];then
            echo "ERROR: uncompress ${remote_host}:${remote_file} failed, please make sure your remote_host ip is right."
            return 1
        fi

        ret=`${DDK_BIN}/IDE-daemon-client --host ${remote_host}:${remote_port} --hostcmd "rm ${remote_file}"`
        if [[ $? -ne 0 ]];then
            echo "ERROR: delete ${remote_host}:${remote_file} failed, please make sure your remote_host ip is right."
            return 1
        fi
    fi
    return 0

}


function check_remote_host()
{
    #check format of remost_host ip
    remote_host=`cat ${script_path}/../src/param_configure.conf | grep "remote_host" | awk -F'[ =]+' '{print $2}'`
    if [[ ${remote_host} = "" ]];then
        echo "please check your param_configure.conf to make sure that each parameter has a value"
        return 1
    fi
    check_ip_addr ${remote_host}
    if [ $? -ne 0 ];then
        echo "ERROR: invalid remote_host ip, please check your settings in configuration file"
        return 1
    fi 
}


# ************************parse presenter_altasdk ip****************************
# Description:  parse presenter_altasdk ip right or not
# $1: remote_host ip
# ******************************************************************************

function parse_presenter_altasdk_ip()
{
    valid_ips=""
    remote_host=$1
    for ip_info in `/sbin/ip addr | grep "inet " | awk -F ' ' '{print $2}'`
    do
        ip=`echo ${ip_info} | awk -F '/' '{print $1}'`
        cidr=`echo ${ip_info} | awk -F '/' '{print $2}'`

        valid_ips="${valid_ips}\t${ip}\n"
        mask=`cidr2mask ${cidr}`
        if [[ ${ip}"X" == "X" ]];then
            continue
        fi
        check_ips_in_same_segment ${ip} ${mask} ${remote_host}
        if [[ $? -eq 0 ]];then
            presenter_atlasdk_ip=${ip}
            echo "Find ${presenter_atlasdk_ip} which is in the same segment with ${remote_host}."
            break
        fi
    done

    
    if [[ ${presenter_atlasdk_ip}"X" != "X" ]];then
        return 0
    fi
    
    echo "Can not find ip in the same segment with ${remote_host}."
    while [[ ${presenter_atlasdk_ip}"X" == "X" ]]
    do
        echo -en "Current environment valid ip list:\n${valid_ips}Please choose one which can connect to Atlas DK Developerment Board:"
        read presenter_atlasdk_ip
        if [[ ${presenter_atlasdk_ip}"X" != "X" ]];then
            check_ip_addr ${presenter_atlasdk_ip}
            if [[ $? -ne 0 ]];then
                echo "Invlid ip, please choose again..."
                presenter_atlasdk_ip=""
            else
                #使用grep检测字段，如果没有找到相应的字段，使用$?会返回非零值
                ret=`/sbin/ifconfig | grep ${presenter_atlasdk_ip}`
                if [[ $? -ne 0 ]];then
                    presenter_atlasdk_ip=""
                fi
            fi
        fi
    done
    return 0
}

function parse_presenter_view_ip()
{
    valid_view_ips=""
    for ip_info in `/sbin/ip addr | grep "inet " | awk -F ' ' '{print $2}'`
    do
        ip=`echo ${ip_info} | awk -F '/' '{print $1}'`
        valid_view_ips="${valid_view_ips}\t${ip}\n"
    done

    while [[ ${presenter_view_ip}"X" == "X" ]]
    do
        echo -en "Current environment valid ip list:\n${valid_view_ips}Please choose one to show the presenter in browser(default: 127.0.0.1):"
        read presenter_view_ip
        
        if [[ ${presenter_view_ip}"X" != "X" ]];then
            check_ip_addr ${presenter_view_ip}
            if [[ $? -ne 0 ]];then
                echo "Invlid ip, please choose again..."
                presenter_view_ip=""
            else
                ret=`/sbin/ifconfig | grep ${presenter_view_ip}`
                if [[ $? -ne 0 ]];then
                    echo "Invlid ip, please choose again..."
                    presenter_view_ip=""
                fi
            fi
        else
            presenter_view_ip="127.0.0.1"
        fi
    done
    return 0
}


# ************************check ip in same segment or not****************************************
# Description:  check ip in same segment or not
# $1: checked ip
# $2: compared ip
# $3: compared net mask
# ******************************************************************************
function check_ips_in_same_segment()
{
    ip=$1
    mask=$2
    remote_host=$3

    OLD_IFS_IP="${IFS}"
    IFS="."
    remote_host_attr=(${remote_host})
    ip_attr=(${ip})
    mask_attr=(${mask})
    IFS=${OLD_IFS_IP}
    for i in `seq 0 3`
    do
        ((calc_remote=${remote_host_attr[${i}]}&${mask_attr[${i}]}))
        ((calc_ip=${ip_attr[${i}]}&${mask_attr[${i}]}))

        if [[ calc_remote -ne calc_ip ]];then
            return 1
        fi
    done
    return 0
}


# ************************convert CIDR to netmask****************************************
# Description:  convert CIDR to netmask
# $1: CIDR
# ******************************************************************************
function cidr2mask()
{
   # Number of args to shift, 255..255, first non-255 byte, zeroes
   set -- $(( 5 - ($1 / 8) )) 255 255 255 255 $(( (255 << (8 - ($1 % 8))) & 255 )) 0 0 0
   [ $1 -gt 1 ] && shift $1 || shift
   echo ${1-0}.${2-0}.${3-0}.${4-0}
}

# ************************check ip****************************************
# Description:  check ip valid or not
# $1: ip
# ******************************************************************************

function check_ip_addr()
{
    ip_addr=$1
    echo ${ip_addr} | grep "^[0-9]\{1,3\}\.\([0-9]\{1,3\}\.\)\{2\}[0-9]\{1,3\}$" > /dev/null
    if [ $? -ne 0 ]
    then
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

# ************************is_proto_version_match********************************
# Description:  check proto code file version is match with protoc version
# $1: proto code head file
# ******************************************************************************
function is_proto_version_match()
{
    file=$1
    if [ ! -f $file ];then
        echo "Error: check proto version failed for $file is not exist"
        return 0
    fi

	min_version=$(grep "PROTOBUF_VERSION" $file | awk -F ' ' '{print $4}' | sed 's/00/./g')
    if [ -z "$min_version" ]; then
        echo "Get min version from $file failed"
        return 0
    fi
    echo "min verison $min_version"

	max_version=$(grep "PROTOBUF_MIN_PROTOC_VERSION" $file | awk -F ' ' '{print $2}' | sed 's/00/./g')
    if [ -z "$max_version" ]; then
        echo "Get max version from $file failed"
        return 0
    fi
    echo "max verison $max_version"

	protoc_version=$($PROTOC --version | awk -F ' ' '{print $2}')
    if [ -z "$protoc_version" ]; then
        echo "Get protoc version from $file failed"
        return 0
    fi
    echo "protoc verison $protoc_version"

	if [[ $protoc_version == $min_version ]] || [[ $protoc_version == $max_version ]] ||\
       ([[ $protoc_version > $min_version ]] && [[ $protoc_version < $max_version ]]); then
		return 1
	fi
    
	return 0
}	

# ************************generate_proto_code***********************************
# Description:  generate proto code by protoc
# $1: proto file
# ******************************************************************************
function generate_proto_code()
{
    proto_file=$1

    cur_dir=$(pwd)
    proto_dir=$(dirname $proto_file)
    filename=$(basename $proto_file)

    cd $proto_dir
	$PROTOC $filename --cpp_out=./
    if [ $? -ne 0 ]
    then
        echo "ERROR: execute $PROTOC $proto_file --cpp_out=$proto_out_dir failed"
        return 1
    fi
    cd $cur_dir

    return 0
}

# ************************check_proto_version***********************************
# Description:  check proto code version, regenerate if version not match
# $1: proto head file
# $2: proto file
# ******************************************************************************
function check_proto_version()
{
    pb_h_file=$1
    proto_file=$2

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$DDK_HOME/lib/x86_64-linux-gcc5.4/"

    echo "check_proto_version $pb_h_file"
    is_proto_version_match $pb_h_file
	if [ $? -eq 1 ];then
        echo "$pb_h_file match the protoc version"
        return 0
    fi       

    echo "The proto code does not match the protoc version, need regenerate code"
    if [ -e "$HOME/ascend_ddk/presenteragent/out/libpresenteragent.so" ];then
        echo "libpresenteragent.so is removed because presenteragent needs to be recompiled"
        rm $HOME/ascend_ddk/presenteragent/out/libpresenteragent.so
    fi
    generate_proto_code $proto_file
    if [ $? -eq 1 ];then
        echo "ERROR: regenerate proto code failed"
        return 1
    fi

    echo "Regenerate proto code success"
    return 0   
}