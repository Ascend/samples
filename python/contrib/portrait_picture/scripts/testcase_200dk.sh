tensorflow_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/portrait.pb"
aipp_cfg="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/insert_op.cfg"
davinci_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/portrait.om"
model_name="portrait"

version=$1

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/"
project_name="portrait_person_pic"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    #mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"ori.jpg"  ${data_source}"ori.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download ori.jpg failed, please check Network."
        return 1
    fi
 
    wget -O ${project_path}/data/"background.jpg"  ${data_source}"background.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download background.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/
    
    wget -O ${project_path}/verify_image/mask_ori.jpg ${verify_source}"mask_ori.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download mask_ori.jpg failed, please check Network."
        return 1
    fi

    return 0
}

function setAtcEnv() {
    # set enviroment param
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
        echo $1
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

    wget -O ${project_path}/model/${tensorflow_model##*/} ${tensorflow_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tensorflow_model failed, please check Network."
        return 1
    fi


    wget -O ${project_path}/model/${aipp_cfg##*/}  ${aipp_cfg} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install aipp config failed, please check Network."
        return 1
    fi

    return 0
}

function getOmModel() {
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]]; then
        # downloadmodel
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return 1
        fi

        atc --model=${project_path}/model/${tensorflow_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --output=${HOME}/models/${project_name}/${model_name}  --output_type=FP32 --input_shape="Inputs/x_input:1,224,224,3" --framework=3 --soc_version=Ascend310
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return 1
        fi
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]]; then 
        wget -O ${HOME}/models/${project_name}/${model_name}".om" ${davinci_model} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "download davinci_model failed, please check Network."
            return 1
        fi
    fi

    return 0
}

function main() {
    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # download data
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        # set model convert param 
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        cd ${project_path}
        getOmModel
        if [ $? -ne 0 ];then
            echo "ERROR: get om model failed"
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

    # reset enviroment 
    setRunEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set executable program running environment failed"
        return ${inferenceError}
    fi

    # run
    cd ${project_path}/src/
    python3.6 main.py 
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    # verify
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/outputs" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/outputs/mask/ -name "*${tmp#*_}"`;do
            echo ${test_file} ${outimage}
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
