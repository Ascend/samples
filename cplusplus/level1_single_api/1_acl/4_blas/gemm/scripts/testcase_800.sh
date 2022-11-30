project_name="cplusplus_acl_execute_gemm"
model_name="0_GEMM_1_2_16_16_1_2_16_16_1_2_16_16_1_2_1_2_1_2_16_16"
version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


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
        export ASCEND_OPP_PATH=${install_path}/opp    
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH    
        export LD_LIBRARY_PATH=${install_path}/atc/lib64
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH    
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


function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    mkdir -p ${project_path}/run/out/op_models
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 

        # 设置模型转换的环境变量
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        # 转模型
        cd ${project_path}/
        atc --singleop=${project_path}/run/out/test_data/config/gemm.json --soc_version=Ascend910 --output=${HOME}/models/${project_name}           
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        cp ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/run/out/op_models/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        cp ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/run/out/op_models/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    fi

    cd ${project_path}/run/out/test_data/data
    python3.7.5 generate_data.py
    if [[ $? -ne 0 ]] || [[ ! -f ${project_path}/run/out/test_data/data/matrix_a.bin ]] || [[ ! -f ${project_path}/run/out/test_data/data/matrix_b.bin ]] || [[ ! -f ${project_path}/run/out/test_data/data/matrix_c.bin ]];then
        echo "ERROR: generate input data failed. please check your project"
        return ${inferenceError}       
    fi

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

    cd ${project_path}/run/out

    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=${HOME}/Ascend/nnrt/latest/runtime/lib64
    
    chmod +x execute_gemm_op
    if [ $? -ne 0 ];then
        echo "ERROR: chmod +x to executable program failed. please check your project"
        return ${inferenceError}
    fi

    ./execute_gemm_op
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    if [[ ! -f "${project_path}/run/out/result_files/matrix_c.bin" ]];then
        echo "ERROR: not find result file!"
        return ${inferenceError}
    fi

    python3 ${script_path}/verify_result.py "${project_path}/run/out/test_data/data/output.bin" "${project_path}/run/out/result_files/matrix_c.bin"
    if [ $? -ne 0 ];then
        echo "ERROR: The result of reasoning is wrong!"
        return ${verifyResError}
    fi   

    echo "run success"

    return ${success}
}
main

