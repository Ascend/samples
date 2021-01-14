tensorflow_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com:443/003_Atc_Models/AE/ATC%20Model/garbage/mobilenetv2.air"
davinci_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com:443/003_Atc_Models/AE/ATC%20Model/garbage/mobilenetv2.air"
model_name="deeplabv3"

version=$1

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/deeplabv3/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/deeplabv3/"
project_name="deeplabv3_pascal_pic"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"dirtycloth.jpg"  ${data_source}"dirtycloth.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download dirtycloth.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"bottle.jpg"  ${data_source}"bottle.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download bottle.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"newspaper.jpg"  ${data_source}"newspaper.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download newspaper.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/out_dirtycloth.jpg ${verify_source}"out_dirtycloth.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_dirtycloth.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/out_bottle.jpg ${verify_source}"out_bottle.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_bottle.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/out_newspaper.jpg ${verify_source}"out_newspaper.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_newspaper.jpg failed, please check Network."
        return 1
    fi

    return 0
}


function setAtcEnv() {
    # set enviroment param
    if [[ ${version} = "c75" ]] || [[ ${version} = "C73" ]];then
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

function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${tensorflow_model##*/} ${tensorflow_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install mindspore_model failed, please check Network."
        return 1
    fi

    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # download test data
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        # download origin model
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

        # set enviroment param
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        # convert model
        cd ${project_path}/model/
        atc --model=${project_path}/model/${mindspore_model##*/} --framework=1 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --input_shape="data:1,3,224,224" --input_format=NCHW
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

    # reconfigure enviroment param
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}
    export PYTHONPATH=/home/HwHiAiUser/Ascend/nnrt/latest/pyACL/python/site-packages/acl:${PYTHONPATH}

    # excute program
    python3.6 ${project_path}/src/classify_test.py ${project_path}/data
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/outputs" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/outputs -name "*${tmp#*_}"`;do
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
