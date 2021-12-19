#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${ScriptPath}
. ${THIRDPART_PATH}/common/sample_common.sh
conf_file_name="param.conf"
function run_presenter()
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

  bash run_presenter_server.sh> /dev/null
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
    echo "[INFO] The program runs successfully, Please visit http://${presenter_view_ip}:7009 for display server!"
    read -p "Enter any command to stop the application:" presenter_view_ip
    kill_run "all"
    return 0
  fi
}
function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main &"
    parse_presenter_view_ip
    if [ $? -ne 0 ];then
      return 1
    fi
    # start runing
    run_presenter
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    return ${success}  
}
main
