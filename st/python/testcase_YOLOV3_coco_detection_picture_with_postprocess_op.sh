pb1="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/yolov3_tensorflow_1.5.pb"
caffe_model1="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt1="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/yolov3_modify.prototxt"
aipp_cfg="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/aipp_nv12.cfg"
model_name1="yolov3_tf_aipp"
model_name2="yolov3_output_op"

data_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/data/"
data_name="test.jpg"
verify_source="https://obs-9be7.obs.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_postprocess_op/verify/"
verify_name="verify_test.jpg"
verify_name2="verify2_test.jpg"
project_name="python_YOLOV3_coco_detection_picture_with_postprocess_op"
script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture_with_postprocess_op
common_script_dir=${project_path}/../../../../common/
run_command="python3.6 object_detect.py ../data"
out_nodes_txt="yolov3/yolov3_head/Conv_6/BiasAdd:0;yolov3/yolov3_head/Conv_14/BiasAdd:0;yolov3/yolov3_head/Conv_22/BiasAdd:0"
model_atc1="atc --model=${project_path}/model/${pb1##*/} --framework=3 --output=${HOME}/models/${project_name}/${model_name1} --output_type=FP32 --input_shape="input:1,416,416,3" --out_nodes=${out_nodes_txt} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --soc_version=Ascend310"
model_atc2="atc --model=${project_path}/model/${caffe_prototxt1##*/} --weight=${project_path}/model/${caffe_model1##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name2} --soc_version=Ascend310"

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    downloadVerifySource2
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    # 转模型
    tf_model=${pb1}
    model_name=${model_name1}
    model_atc=${model_atc1}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    tf_model=" "
    caffe_model=${caffe_model1}
    caffe_prototxt=${caffe_prototxt1}
    model_name=${model_name2}
    model_atc=${model_atc2}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    verify_ret=1
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out -name "*${tmp#*_}"`;do
            python3.6 ${common_script_dir}/verify_result.py ${test_file} ${outimage}
            if [ $? -eq 0 ];then
		verify_ret=0
            fi
        done
    done
    if [ ${verify_ret} -ne 0 ];then
	echo "ERROR: The result of reasoning is wrong!"
    fi

    #run_picture_python
    #ret=$?
    #if [ $? -ne 0 ];then
    #    return ${ret}
    #fi

    return ${success}
}
main


