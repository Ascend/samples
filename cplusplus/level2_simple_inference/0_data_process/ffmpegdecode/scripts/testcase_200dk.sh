data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/ffmpegdecode/cat.mp4"
project_name="ffmpeg_decode"

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
}

function buildLibAtlasUtil() {
	cd ${project_path}/../../../common/atlasutil/
	make
	if [ $? -ne 0 ];then
        echo "ERROR: make atlasutil failed."
        return ${inferenceError}
    fi
	
	make install
	if [ $? -ne 0 ];then
        echo "ERROR: make install atlasutil failed."
        return ${inferenceError}
    fi
}

function downloadDataWithVerifySource(){
    
    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/cat.mp4  ${data_source} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download cat.mp4 failed, please check Network."
        return 1
    fi

    return 0
}


function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi

    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    buildLibAtlasUtil
	if [ $? -ne 0 ];then
        echo "ERROR: build libatlasutil.so failed"
        return ${inferenceError}
    fi    

    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

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

    # 运行程序
    mv ${project_path}/out/main ${project_path}/out/${project_name}

    ./${project_name} ${project_path}/data/cat.mp4 
    echo "run success"
    return ${success}
}

main
