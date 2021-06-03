air_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air"
model_name="mnist"

version=$1

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/verify_file/"
project_name="lenet_mindspore_picture"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"test1.png"  ${data_source}"test1.png"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test1.png failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"test2.png"  ${data_source}"test2.png"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test2.png failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"test3.png"  ${data_source}"test3.png"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test3.png failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/verify_test1.txt ${verify_source}"verify_test1.txt" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test1.txt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/verify_test2.txt ${verify_source}"verify_test2.txt" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test2.txt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/verify_test3.txt ${verify_source}"verify_test3.txt" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test3.txt failed, please check Network."
        return 1
    fi

    return 0
}


function setAtcEnv() {
    
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

function setRunEnv() {

    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl:${PYTHONPATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux/pyACL/python/site-packages/acl:${PYTHONPATH}
    fi

    return 0
}


function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

   

    wget -O ${project_path}/model/${air_model##*/} ${air_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install air_model failed, please check Network."
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

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

                
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        # convert model     
        cd ${project_path}/model/
		atc --model=${project_path}/model/${air_model##*/} --framework=1 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
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

    cd ${project_path}

        
    setRunEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set executable program running environment failed"
        return ${inferenceError}
    fi

    
    python3.6 ${project_path}/src/classify.py ${project_path}/data
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
        
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/outputs" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/outputs -name "*${tmp#*_}"`;do
            python3.6 ${script_path}/verify_result.py ${test_file} ${outimage}
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
