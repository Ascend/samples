tensorflow_model1="https://c7xcode.obs.cn-north-4.myhuaweicloud.com:443/models/scenetextrecognition/advancedeast.pb"
tensorflow_model2="https://c7xcode.obs.cn-north-4.myhuaweicloud.com:443/models/scenetextrecognition/chineseocr.pb"
model_name1="advancedeast"
model_name2="chineseocr"
version=$1

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/scenetextrecognition/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/scenetextrecognition/"
project_name="scenetextrecognition"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/src/Data/image/
    wget -O ${project_path}/src/Data/image/"test.jpg"  ${data_source}"test.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/test_result.txt ${verify_source}"test_result.txt" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test_result.txt failed, please check Network."
        return 1
    fi

    return 0
}


function setAtcEnv() {
    # set enviroment param
    if [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then    
        export HOME=/home/HwHiAiUser
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}        
        echo "setAtcEnv success."
    fi

    return 0
}

function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    wget -O ${project_path}/src/Data/Models/TextRecognition/${tensorflow_model2##*/} ${tensorflow_model2} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tensorflow_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/src/Data/Models/TextDetection/${tensorflow_model1##*/} ${tensorflow_model1} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tensorflow_model failed, please check Network."
        return 1
    fi
    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # reconfigure enviroment param
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}
    export ASCEND_HOME=/home/HwHiAiUser/Ascend     
    export LD_LIBRARY_PATH=$ASCEND_HOME/ascend-toolkit/latest/acllib/lib64:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/usr/local/opencv/lib64:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/usr/local/opencv/lib:$LD_LIBRARY_PATH
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

        cd ${project_path}/src/Data/Models/TextDetection
        atc --model=./advancedeast.pb \
        --framework=3 \
        --output=./advancedeast \
        --soc_version=Ascend310 \
        --insert_op_conf=./advancedeast_aipp.cfg \
        --input_shape="input_img:1,-1,-1,3" \
        --dynamic_image_size="832,832;416,832;832,416;416,416"
        if [ $? -ne 0 ];then
            echo "ERROR: convert model advancedeast failed"
            return ${inferenceError}
        fi
        
        cd ${project_path}/src/Data/Models/TextRecognition
        atc --model=./chineseocr.pb \
        --framework=3 \
        --output=./chineseocr \
        --soc_version=Ascend310 \
        --insert_op_conf=./chineseocr_aipp.cfg \
        --input_shape="the_input:1,-1,-1,1" \
        --dynamic_image_size="32,32;32,64;32,96;32,128;32,160;32,192;32,224;32,256;32,288;32,320"
        if [ $? -ne 0 ];then
            echo "ERROR: convert model chineseocr failed"
            return ${inferenceError}
        fi
        
    fi
    
    # download data
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi
    cd ${project_path}

    bash ${project_path}/src/build.sh
    if [ $? -ne 0 ];then
        echo "ERROR: build failed. please check your project"
        return ${inferenceError}
    fi   

    cp  -fr ${project_path}/src/Data/image ${project_path}/src/dist 
    # excute program
    cd ${project_path}/src/dist/
    ${project_path}/src/dist/ocr 
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    # verify
    for outimage in $(find ${project_path}/verify_image -name "*.txt");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/src/dist/result" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/src/dist/result -name "*${tmp#*_}"`;do
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
