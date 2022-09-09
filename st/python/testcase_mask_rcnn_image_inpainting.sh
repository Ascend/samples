ms_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/maskrcnn_mindspore.air"
ms_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/aipp_rgb.cfg"
model_name1="maskrcnn_mindspore_rgb"

js_model="https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json"
mat_mul_name="0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648"

offline_model="https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/"
data_name="photo1.jpg"

verify_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/verify/"
verify_name="verify_test.jpg"

project_name="python_mask_rcnn_image_inpainting"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/level3_multi_model/mask_rcnn_image_inpainting/scripts
project_path=${script_path_temp}/../../python/level3_multi_model/mask_rcnn_image_inpainting
common_script_dir=${project_path}/../../../common/
run_command="python3.6 mask_rcnn.py 410 664"

input_txt="x:1,3,768,1280;im_info:1,4"
model_atc1="atc --input_format=NCHW --framework=1 --model=${project_path}/model/${ms_model##*/} --input_shape=${input_txt} --output=${HOME}/models/${project_name}/${model_name1} --insert_op_conf=${project_path}/model/${ms_cfg##*/} --precision_mode=allow_fp32_to_fp16 --soc_version=Ascend310 --op_select_implmode=high_precision"
model_atc2="atc --singleop=matmul_27648.json --output=${HOME}/models/${project_name}/ --soc_version=Ascend310"

. ${common_script_dir}/testcase_common.sh

function run_mask_rcnn_image_inpainting()
{
    cd ${project_path}/src
    ${run_command}
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


function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    mindspore_model=${ms_model}
    aipp_cfg=${ms_cfg}
    model_name=${model_name1}
    model_atc=${model_atc1}
    echo ${aipp_cfg}
    echo ${model_name}
    echo ${model_atc}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    mindspore_model=""
    aipp_cfg=""
    json_model=${js_model}
    model_name=${mat_mul_name}
    model_atc=${model_atc2}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    
    wget -O ${project_path}/model/${offline_model##*/} ${offline_model} --no-check-certificate
    
    run_mask_rcnn_image_inpainting
    ret=$?
    if [ $? -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main



