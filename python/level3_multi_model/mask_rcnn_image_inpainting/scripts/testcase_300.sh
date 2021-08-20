ms_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/maskrcnn_mindspore.air"
ms_aipp="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/aipp_rgb.cfg"
json_model="https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json"

mat_mul_name="0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648"
model_name="mask_rcnn_image_inpainting"

version=$1

data_source="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/photo1.jpg"

verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/imageinpainting_hifill/verify_image/"

project_name="python_mask_rcnn_image_inpainting"
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadData() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"photo1.jpg"  ${data_source}  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download photo1.jpg failed, please check Network."
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
    elif [[ ${version} = "c77" ]] || [[ ${version} = "C77" ]];then
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

    wget -O ${project_path}/model/${ms_model##*/} ${ms_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install ms_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${ms_aipp##*/} ${ms_aipp} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install ms_aipp failed, please check Network."
        return 1
    fi
	
	wget -O ${project_path}/model/${json_model##*/} ${json_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install json_model failed, please check Network."
        return 1
    fi

    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # 下载测试集
    downloadData
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        # 下载原始模型文件
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
		
		atc --input_format=NCHW --framework=1 --model=${project_path}/model/${ms_model##*/} --input_shape="x:1, 3, 768, 1280; im_info: 1, 4" --output=${HOME}/models/${project_name}/maskrcnn_mindspore_rgb \
            --insert_op_conf=${project_path}/model/${ms_aipp##*/} --precision_mode=allow_fp32_to_fp16 --soc_version=Ascend310 --op_select_implmode=high_precision
		
		atc --singleop=matmul_27648.json --output=${HOME}/models/${project_name}/${mat_mul_name} --soc_version=Ascend310
        #cd ${HOME}/models/${project_name}/
        wget  https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om --no-check-certificate

        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/maskrcnn_mindspore_rgb".om" ${project_path}/model/maskrcnn_mindspore_rgb".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
		ln -s ${HOME}/models/${project_name}/${mat_mul_name}/${mat_mul_name}".om" ${project_path}/model/${mat_mul_name}".om"
        #cp -r ${HOME}/models/${project_name}/${mat_mul_name}".om" ${project_path}/model/${mat_mul_name}".om"
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
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}
    export PYTHONPATH=/home/HwHiAiUser/Ascend/nnrt/latest/pyACL/python/site-packages/acl:${PYTHONPATH}

    # 运行程序
    python3.7  mask_rcnn.py 410 664
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/output" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/output -name "*${tmp#*_}"`;do
            python3.7 ${script_path}/verify_result.py ${test_file} ${outimage}
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
