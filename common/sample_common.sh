#!/bin/bash
function check_ip_addr()
{
  ip_addr=$1
  echo ${ip_addr} | grep "^[0-9]\{1,3\}\.\([0-9]\{1,3\}\.\)\{2\}[0-9]\{1,3\}$" > /dev/null
  if [ $? -ne 0 ];then
    return 1
  fi

  for num in `echo ${ip_addr} | sed "s/./ /g"`
  do
    if [ $num -gt 255 ] || [ $num -lt 0 ];then
      return 1
    fi
  done
  return 0
}
function kill_run()
{
  if [[ ${1} = "only_presenter" ]];then
    presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep "display" | awk -F ' ' '{print $2}'`
    if [[ ${presenter_server_pid}"X" != "X" ]];then
      echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
      kill -9 ${presenter_server_pid}
      if [ $? -ne 0 ];then
        echo "ERROR: kill presenter server process failed."
        return 1
      else
        echo "[INFO] kill presenter server process success"
      fi
    else
      echo "[ERROR] presenter server process not exists"
      return 1
    fi
    return 0
  else
    project_pid=`ps -ef | grep "${running_command}" | grep "${data_command}" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
      echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
      kill -9 ${project_pid}
      if [ $? -ne 0 ];then
        echo "ERROR: kill project process failed."
        return 1
      fi

      presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep "display" | awk -F ' ' '{print $2}'`
      if [[ ${presenter_server_pid}"X" != "X" ]];then
        echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
        kill -9 ${presenter_server_pid}
        if [ $? -ne 0 ];then
          echo "ERROR: kill presenter server process failed."
          return 1
        fi
      else
        echo "[ERROR] presenter server process not exists"
	return 1
      fi
    else
      echo "[ERROR] project process not exists"
      return 1
    fi
  fi
}
function find_model()
{
  ret=`find ${ModelPath} -maxdepth 1 -name $1 2> /dev/null`
  if [[ ${ret} ]];then
    echo "[INFO] The $1 already exists.start buiding"
    return 0
  else
    echo "[ERROR] $1 does not exist, please follow the readme to convert the model and place it in the correct position!"
    return 1
  fi
}
function target_kernel()
{
  declare -i CHOICE_TIMES=0
  while [[ ${TargetKernel}"X" = "X" ]]
  do
    # three times choice 
    [[ ${CHOICE_TIMES} -ge 3 ]] && break || ((CHOICE_TIMES++))
    read -p "please input TargetKernel? [arm/x86]:" TargetKernel
    if [ ${TargetKernel}"z" = "armz" ] || [ ${TargetKernel}"z" = "Armz" ] || [ ${TargetKernel}"z" = "x86z" ] || [ ${TargetKernel}"z" = "X86z" ]; then
      echo "[INFO] input is normal, start preparation."
    else
      echo "[WARNING] The ${CHOICE_TIMES}th parameter input error!"
      TargetKernel=""
    fi
  done
  if [ ${TargetKernel}"z" = "z" ];then
    echo "[ERROR] TargetKernel entered incorrectly three times, please input arm/x86!"
    return 1
  else
    return 0
  fi
}
function build()
{
  UserKernel=`arch`
  if [[ ${TargetKernel} = "x86" ]] || [[ ${TargetKernel} = "X86" ]];then
    TargetCompiler="g++"
    TargetKernel="x86"
  else
    if [[ ${UserKernel} == "x86_64" ]];then
      TargetCompiler="aarch64-linux-gnu-g++"
      TargetKernel="arm"
    else
      TargetCompiler="g++"
      TargetKernel="arm"
    fi
  fi
  if [ -d ${ScriptPath}/../build/intermediates/host ];then
    rm -rf ${ScriptPath}/../build/intermediates/host
  fi
    
  mkdir -p ${ScriptPath}/../build/intermediates/host
  cd ${ScriptPath}/../build/intermediates/host

  # Start compiling
  cmake ../../../src -DCMAKE_CXX_COMPILER=${TargetCompiler} -DCMAKE_SKIP_RPATH=TRUE
  if [ $? -ne 0 ];then
    echo "[ERROR] cmake error, Please check your environment!"
    return 1
  fi
  make
  if [ $? -ne 0 ];then
    echo "[ERROR] build failed, Please check your environment!"
    return 1
  fi
  cd - > /dev/null
}

function parse_presenter_view_ip()
{
  valid_view_ips=""
  for ip_info in `/sbin/ip addr | grep "inet " | awk -F ' ' '{print $2}'`
  do
      ip=`echo ${ip_info} | awk -F '/' '{print $1}'`
      valid_view_ips="${valid_view_ips}\t${ip}\n"
  done

  declare -i CHOICE_TIMES=0
  while [[ ${presenter_view_ip}"X" == "X" ]]
  do
    [[ ${CHOICE_TIMES} -ge 3 ]] && break || ((CHOICE_TIMES++))
    echo -en "Current environment valid ip list:\n${valid_view_ips}Please choose one to show the presenter in browser:"
    read presenter_view_ip
        
    if [[ ${presenter_view_ip}"X" != "X" ]];then
      check_ip_addr ${presenter_view_ip}
      if [[ $? -ne 0 ]];then
        echo "[WARNING] The ${CHOICE_TIMES}th input Invlid ip, please choose again..."
        presenter_view_ip=""
      else
        ret=`/sbin/ifconfig | grep ${presenter_view_ip}`
        if [[ $? -ne 0 ]];then
          echo "[WARNING] The ${CHOICE_TIMES}th input Invlid ip, please choose again..."
          presenter_view_ip=""
        fi
      fi
    else
      echo "[WARNING] The ${CHOICE_TIMES}th input is empty, please choose again..."
      presenter_view_ip=""
    fi
  done

  if [ ${presenter_view_ip}"z" = "z" ];then
    echo "[ERROR] presenter_view_ip entered incorrectly three times!"
    return 1
  else
    return 0
  fi
}

function running_presenter()
{
  Kernel=`uname -m`
  if [[ ${Kernel} = "x86_64" ]];then
    TargetKernel="x86"
  else
    TargetKernel="arm"
  fi

  sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${presenter_view_ip}/g" ${ScriptPath}/${conf_file_name}
  sed -i "s/presenter_view_ip=[0-9.]*/presenter_view_ip=${presenter_view_ip}/g" ${ScriptPath}/${conf_file_name}

  cp -f ${ScriptPath}/${conf_file_name} ${ScriptPath}/../out
  
  if [[ ! -n "${common_script_dir}"  ]];then
        cd ${ScriptPath}/../../../../../common
  else
        cd ${common_script_dir}
  fi

  bash run_presenter_server.sh ${ScriptPath}/${conf_file_name} > /dev/null
  if [ $? -ne 0 ];then
    echo "[ERROR] The program failed to run, please check the environmoent!"
    cd ${ScriptPath}
    kill_run
    return 1
  fi

  sleep 2

  cd ${ScriptPath}/../out
  ${running_command} ${data_command} &
  sleep 1

  project_pid=`ps -ef | grep "${running_command}" | grep "${data_command}" | awk -F ' ' '{print $2}'`
  if [[ ${project_pid}"X" = "X" ]];then
    echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
    kill_run "only_presenter"
    return 1
  else
    sleep 3
    echo "[INFO] The program runs successfully, Please visit http://${presenter_view_ip}:7007 for display server!"
    read -p "Enter any command to stop the application:" presenter_view_ip
    kill_run "all"
    return 0
  fi
}
function running_presenter_python()
{
  sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${presenter_view_ip}/g" ${ScriptPath}/${conf_file_name}
  sed -i "s/presenter_view_ip=[0-9.]*/presenter_view_ip=${presenter_view_ip}/g" ${ScriptPath}/${conf_file_name}

  if [[ ! -n "${common_script_dir}"  ]];then
        cd ${ScriptPath}/../../../../../common
  else
        cd ${common_script_dir}
  fi

  bash run_presenter_server.sh ${ScriptPath}/${conf_file_name} > /dev/null
  if [ $? -ne 0 ];then
    echo "[ERROR] The program failed to run, please check the environmoent!"
    cd ${ScriptPath}
    kill_run
    return 1
  fi

  sleep 2

  cd ${ScriptPath}/../src
  ${running_command} ${data_command} &
  sleep 1

  project_pid=`ps -ef | grep "${running_command}" | grep "${data_command}" | awk -F ' ' '{print $2}'`
  if [[ ${project_pid}"X" = "X" ]];then
    echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
    kill_run "only_presenter"
    return 1
  else
    sleep 3
    echo "[INFO] The program runs successfully, Please visit http://${presenter_view_ip}:7007 for display server!"
    read -p "Enter any command to stop the application:" presenter_view_ip
    kill_run "all"
    return 0
  fi
}
function running()
{
    Kernel=`uname -m`
   if [[ ${Kernel} = "x86_64" ]];then
        Targetkernel="x86"
    else
        Targetkernel="arm"
    fi
    cd ${ScriptPath}/../out
    rm -rf output
    mkdir output
    ${running_command}
    if [ $? -ne 0 ];then
        echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
        return 1
    else
        echo "[INFO] The program runs successfully, please view the result file in the ${ScriptPath}/../out/output directory!"
        return 0
    fi
}
