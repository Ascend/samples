yolov3_model1="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
yolov3_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
yolov3_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_picture/aipp_nv12.cfg"

color_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/color.pb"
color_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/aipp.cfg"

model_name_1="yolov3"
model_name_2="color_dvpp_10batch"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car"
project_name="cplusplus_YOLOV3_carColor_sample"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/2_object_detection/YOLOV3_carColor_sample/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common
run_command="./main ../data"
model_atc_1="atc --model=${project_path}/model/${yolov3_prototxt##*/} --weight=${project_path}/model/${yolov3_model1##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name_1} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${yolov3_cfg##*/}"
model_atc_2="atc --input_shape="input_1:10,224,224,3" --output=${HOME}/models/${project_name}/${model_name_2} --soc_version=Ascend310 --framework=3 --model=${project_path}/model/${color_model##*/} --insert_op_conf=${project_path}/model/${color_cfg##*/}"
jpg=".jpg"
. ${common_script_dir}/testcase_common.sh

function main() {

    for ((i=1; i<3; i ++))
    do
    if [ ! -f "${ModelPath}/../data/${i}${jpg}" ];then
        wget -O ${project_path}/data/car$i".jpg"  ${data_source}$i".jpg"  --no-check-certificate
    fi
    if [ $? -ne 0 ];then
            echo "Download jpg failed, please check network."
            return 1
        fi
    done

    # 转模型
    caffe_model=${yolov3_model1}
    caffe_prototxt=${yolov3_prototxt}
    aipp_cfg=${yolov3_cfg}
    model_name=${model_name_1}
    model_atc=${model_atc_1}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    
    caffe_model=""
    caffe_prototxt=""
    tf_model=${color_model}
    aipp_cfg=${color_cfg}
    model_name=${model_name_2}
    model_atc=${model_atc_2}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    ret=$?
    if [ ${ret} -ne 0 ];then
        return ${ret}
    fi

    return ${success}
}
main

