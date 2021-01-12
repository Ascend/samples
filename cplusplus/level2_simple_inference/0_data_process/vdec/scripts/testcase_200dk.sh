data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdec"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdec/verify_image"
project_name="cplusplus_vdec"

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
    fi

    return 0
}

function downloadDataWithVerifySource(){
    
    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"vdec_h265_1frame_rabbit_1280x720.h265"  ${data_source}/"vdec_h265_1frame_rabbit_1280x720.h265" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download vdec_h265_1frame_rabbit_1280x720.h265 failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/"image1.yuv"  ${verify_source}/"image1.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image1.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image2.yuv"  ${verify_source}/"image2.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image2.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image3.yuv"  ${verify_source}/"image3.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image3.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image4.yuv"  ${verify_source}/"image4.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image4.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image5.yuv"  ${verify_source}/"image5.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image5.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image6.yuv"  ${verify_source}/"image6.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image6.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image7.yuv"  ${verify_source}/"image7.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image7.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image8.yuv"  ${verify_source}/"image8.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image8.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image9.yuv"  ${verify_source}/"image9.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image9.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image10.yuv"  ${verify_source}/"image10.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image10.yuv failed, please check Network."
        return 1
    fi

    return 0
}


function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
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

    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}

    mkdir -p ${project_path}/out/output
    # 运行程序
    ./main ${project_path}/data/
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   

    #调用python脚本判断本工程推理结果是否正常
    result=$(ls ${project_path}/out/output/*.yuv 2>/dev/null)
    if [[ ${result}"x" = "x" ]];then
        echo "ERROR: verify failed. please check your project"
        return ${verifyResError}
    fi

    echo "run success"

    return ${success}
}
main
