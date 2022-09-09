caffe_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/retina-unet/vel_hw_iter_5000.caffemodel"
caffe_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/retina-unet/deploy_vel_ascend.prototxt"
model_name="vessel"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vessel_segmentation/test_image/"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/vessel_segmentation/verify_image/"
project_name="gesture_recognition_picture"

data_name="test1.png"
verify_name="verify_test1.jpg"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../python/contrib/vessel_segmentation
project_path=${script_path}
common_script_dir=${script_path}/../../../common/
run_command="python3.6 main.py ../data"
model_atc="atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/}  --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data -output_type=FP32"

. ${common_script_dir}/testcase_common.sh
function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture_python
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}

main
