caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/faster_rcnn/faster_rcnn.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/faster_rcnn/faster_rcnn.prototxt"
model_name="faster_rcnn"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/faste_RCNN_VOC_detection_dynamic_resolution/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/faste_RCNN_VOC_detection_dynamic_resolution/verify_image/"
project_name="cplusplus_faste_RCNN_VOC_detection_dynamic_resolution"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"boat.jpg"  ${data_source}"boat.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test1.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"bus.jpg"  ${data_source}"bus.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test2.jpg failed, please check Network."
        return 1
    fi


    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/"out_boat.jpg" ${verify_source}"out_boat.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test1.jpg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"out_bus.jpg" ${verify_source}"out_bus.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test2.jpg failed, please check Network."
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
         export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}

function downloadOriginalModel() {
    
    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${caffe_model##*/} ${caffe_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_model failed, please check Network."
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
        atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape="data:1,3,-1,-1;im_info:1,3" --dynamic_image_size="512,512;600,600;800,800"
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
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
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
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}

    # 运行程序
    ./main ${project_path}/data
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   

    bus_line=$(awk "match(\$0, \"bus.jpg\") {print NR}" inference_result.txt)
    boat_line=$(awk "match(\$0, \"boat.jpg\") {print NR}" inference_result.txt)

    label1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $1}' | grep label | awk -F'[= ]+' '{print $2}')
    score1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $2}' | grep score | awk -F'[= ]+' '{print $2}')
    ltX_Scale1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $3}' | grep ltX_Scale | awk -F'[= ]+' '{print $2}')
    ltY_Scale1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $4}' | grep ltY_Scale | awk -F'[= ]+' '{print $2}')
    rbX_Scale1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $5}' | grep rbX_Scale | awk -F'[= ]+' '{print $2}')
    rbY_Scale1=$(awk "NR==$((bus_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $6}' | grep rbY_Scale | awk -F'[= ]+' '{print $2}')

    scoreOffset=$((99-${score1}))
    ltXOffset=$(echo "0.115-${ltX_Scale1}" | bc | awk '{printf "%.2f",$0}')
    ltYOffset=$(echo "0.162-${ltY_Scale1}" | bc | awk '{printf "%.2f",$0}')
    rbXOffset=$(echo "0.930-${rbX_Scale1}" | bc | awk '{printf "%.2f",$0}')
    rbYOffset=$(echo "0.898-${rbY_Scale1}" | bc | awk '{printf "%.2f",$0}')

    scoreOffset=${scoreOffset/-/}
    ltXOffset=${ltXOffset/-/}
    ltYOffset=${ltYOffset/-/}
    rbXOffset=${rbXOffset/-/}
    rbYOffset=${rbYOffset/-/}

    if [[ ${label1} != "bus" ]] || [[ ${scoreOffset} -gt 5 ]] || [[ `echo "${ltXOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${ltYOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${rbXOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${rbYOffset} > 0.05" | bc` -eq 1 ]];then
        echo "ERROR: The bus.jpg result of reasoning is wrong!"
        return ${verifyResError}
    fi

    label2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $1}' | grep label | awk -F'[= ]+' '{print $2}')
    score2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $2}' | grep score | awk -F'[= ]+' '{print $2}')
    ltX_Scale2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $3}' | grep ltX_Scale | awk -F'[= ]+' '{print $2}')
    ltY_Scale2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $4}' | grep ltY_Scale | awk -F'[= ]+' '{print $2}')
    rbX_Scale2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $5}' | grep rbX_Scale | awk -F'[= ]+' '{print $2}')
    rbY_Scale2=$(awk "NR==$((boat_line+1))" ../out/inference_result.txt | awk -F'[;]+' '{print $6}' | grep rbY_Scale | awk -F'[= ]+' '{print $2}')
    
    scoreOffset=$((99-${score2}))
    ltXOffset=$(echo "0.100-${ltX_Scale2}" | bc | awk '{printf "%.2f",$0}')
    ltYOffset=$(echo "0.260-${ltY_Scale2}" | bc | awk '{printf "%.2f",$0}')
    rbXOffset=$(echo "0.874-${rbX_Scale2}" | bc | awk '{printf "%.2f",$0}')
    rbYOffset=$(echo "0.694-${rbY_Scale2}" | bc | awk '{printf "%.2f",$0}')

    scoreOffset=${scoreOffset/-/}
    ltXOffset=${ltXOffset/-/}
    ltYOffset=${ltYOffset/-/}
    rbXOffset=${rbXOffset/-/}
    rbYOffset=${rbYOffset/-/}

    if [[ ${label2} != "boat" ]] || [[ ${scoreOffset} -gt 5 ]] || [[ `echo "${ltXOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${ltYOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${rbXOffset} > 0.05" | bc` -eq 1 ]] || [[ `echo "${rbYOffset} > 0.05" | bc` -eq 1 ]];then
        echo "ERROR: The boat.jpg result of reasoning is wrong!"
        return ${verifyResError}
    fi

    echo "run success"

    return ${success}
}
main
