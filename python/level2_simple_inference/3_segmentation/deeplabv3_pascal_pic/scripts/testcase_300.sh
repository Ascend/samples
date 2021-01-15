tensorflow_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com:443/003_Atc_Models/nkxiaolei/DeepLapV3_Plus/deeplabv3_plus.pb"
davinci_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com:443/003_Atc_Models/nkxiaolei/DeepLapV3_Plus/deeplabv3_plus.om"
model_name="deeplabv3_plus"

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

    wget -O ${project_path}/data/"cat.jpg"  ${data_source}"cat.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download cat.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"dog.jpg"  ${data_source}"dog.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download dog.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"girl.jpg"  ${data_source}"girl.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download girl.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/out_dog.jpg ${verify_source}"out_dog.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_dog.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/out_girl.jpg ${verify_source}"out_girl.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_girl.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/out_cat.jpg ${verify_source}"out_cat.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download out_cat.jpg failed, please check Network."
        return 1
    fi

    return 0
}


function setAtcEnv() {
    # set enviroment param
    if [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
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
        echo "install tensorflow_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${davinci_model##*/} ${davinci_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install davinci_model failed, please check Network."
        return 1
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
        # downloadmodel
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

        # set model convert param 
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi
    fi
    cd ${project_path}

    # reconfigure enviroment param
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}
    export PYTHONPATH=/home/HwHiAiUser/Ascend/nnrt/latest/pyACL/python/site-packages/acl:${PYTHONPATH}

    # excute program
    python3.6 ${project_path}/src/deeplabv3.py ${project_path}/data
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
