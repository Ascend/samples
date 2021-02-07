presenter_server_name="display"
project_name="cplusplus_ascendcamera"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2








function setBuildEnv() {
    # 设置代码编译时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c76" ]] || [[ ${version} = "C76" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}



function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi



    # 创建目录用于存放编译文件
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    # 产生Makefile
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "ERROR: cmake failed. please check your project"
        return ${inferenceError}
    fi

    make 
    if [ $? -ne 0 ];then
        echo "ERROR: make failed. please check your project"
        return ${inferenceError}
    fi

    cd ${project_path}/out
    mkdir -p output


    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}


    mv main ${project_name}
    ./${project_name} -i -c 0 -o ${project_path}/out/output/filename.jpg --overwrite 
    result=$(ls ${project_path}/out/output/*.jpg 2>/dev/null)
    if [[ ${result}"x" = "x" ]];then
        echo "ERROR: verify failed. please check your project"
        return ${verifyResError}
    fi

    # rm output/*
    # ./${project_name}  -c 0 -o ${project_path}/out/output/1.mp4
    # result=$(ls ${project_path}/out/output/*.mp4 2>/dev/null)
    # if [[ ${result}"x" = "x" ]];then
    #     echo "ERROR: verify failed. please check your project"
    #     return ${verifyResError}
    # fi

    # 开启presenter server
    # bash ${script_path}/run_presenter_server.sh 
    # if [ $? -ne 0 ];then
    #     echo "ERROR: run presenter server failed. please check your project"
    #     return ${inferenceError}
    # fi

    # sleep 2

    # ./${project_name}  -v -c 0 -t 10 --fps 10 -w 704 -h 576 -s 127.0.0.1:7002/display
    # if [ $? -ne 0 ];then
    #     echo "ERROR: run presenter server failed. please check your project"
    #     return ${inferenceError}
    # fi

    # presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep "${presenter_server_name}" | awk -F ' ' '{print $2}'`
    # if [[ ${presenter_server_pid}"X" != "X" ]];then
    #     echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
    #     kill -9 ${presenter_server_pid}
    #     if [ $? -ne 0 ];then
    #         echo "ERROR: kill presenter server process failed."
    #         return ${inferenceError}
    #     fi
    # fi


    echo "run success"

    return ${success}
}
main
