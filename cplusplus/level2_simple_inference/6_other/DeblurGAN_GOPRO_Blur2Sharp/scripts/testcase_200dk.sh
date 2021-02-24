tf_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeblurGAN/DeblurrGAN-pad-01051648.pb"
model_name="blurtosharp_pad_1280_720"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/DeblurGAN_GOPRO_Blur2Sharp/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/DeblurGAN_GOPRO_Blur2Sharp/verify_image/"
project_name="cplusplus_DeblurGAN_GOPRO_Blur2Sharp"
version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"data.jpg"  ${data_source}"data.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download data.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/verify_data.jpg ${verify_source}"verify_data.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_data.jpg failed, please check Network."
        return 1
    fi

    return 0
}


function setAtcEnv() {
    # 设置模型转换时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export PYTHONPATH=${install_path}/atc/python/site-packages/te:${install_path}/atc/python/site-packages/topi:$PYTHONPATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    fi

    return 0
}

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

function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${tf_model##*/} ${tf_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tf_model failed, please check Network."
        return 1
    fi

    return 0
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

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        # 下载原始模型文件[aipp_cfg文件]
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi
    
        # 设置模型转换的环境变量
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        # 转模型
        cd ${project_path}/model/
        if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
            atc --input_shape="blur:1,720,1280,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=3 --model=${project_path}/model/${tf_model##*/} --log=info
            if [ $? -ne 0 ];then
                echo "ERROR: convert model failed"
                return ${inferenceError}
            fi
        elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
            cd  ${HOME}/models/${project_name}/
            wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/DeblurGAN/blurtosharp_pad_1280_720.om --no-check-certificate
        fi

        ln -s ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -s ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
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

    # 创建目录用于存放编译文件
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

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

    # 运行程序
    mkdir output

    ./main ../data
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out/output" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out/output -name "*${tmp#*_}"`;do
            python3 ${script_path}/verify_result.py ${test_file} ${outimage}
            if [ $? -ne 0 ];then
                echo "ERROR: The result of reasoning is wrong!"
                return ${verifyResError}
            fi   
        done
    done

    echo "run success"


    return ${success}
}
main


