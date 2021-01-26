caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
aipp_cfg="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/aipp_nv12.cfg"
model_name="yolov3_yuv"
    
    
version=$1

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_picture-python/test_image/"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_picture-python/verify_image/"
project_name="python_YOLOV3_coco_detection_picture-python"

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"test.jpg"  ${data_source}"test.jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download test.jpg failed, please check Network."
        return 1
    fi

    mkdir -p ${project_path}/verify_image/

    wget -O ${project_path}/verify_image/verify_test.jpg ${verify_source}"verify_test.jpg" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download verify_test.jpg failed, please check Network."
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

    wget -O ${project_path}/model/${aipp_cfg##*/}  ${aipp_cfg} --no-check-certificate
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
        atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/}
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

    cd ${project_path}/src

    # 重新配置程序运行所需的环境变量
    setRunEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set executable program running environment failed"
        return ${inferenceError}
    fi

    # 运行程序
    python3.6 object_detect.py ../data
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
