project_pid=`ps -ef | grep "./main ../data/black-white_video.mp4" | awk -F ' ' '{print $2}'`
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
    fi
else 
    return 0
fi
