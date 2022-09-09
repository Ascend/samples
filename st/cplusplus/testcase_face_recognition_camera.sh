face_detection_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel"
face_detection_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt"
face_detection_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_recognition_camera/face_detection_insert_op.cfg"
vanillacnn_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vanillacnn/vanillacnn.caffemodel"
vanillacnn_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vanillacnn/vanilla_deploy.prototxt"
sphereface_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sphereface/sphereface.caffemodel"
sphereface_prototxt="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sphereface/sphereface.prototxt"
sphereface_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_recognition_camera/sphereface_insert_op.cfg"
model_name1="face_detection"
model_name2="vanillacnn"
model_name3="sphereface"
presenter_server_name="facial_recognition"
conf_file_name="param.conf"
project_name="cplusplus_face_facial_recognition"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/scripts
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../../common/
model_atc1="atc --model=${project_path}/model/${face_detection_prototxt##*/} --weight=${project_path}/model/${face_detection_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name1} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${face_detection_cfg##*/} --input_shape="data:1,3,300,300" --input_format=NCHW"
model_atc2="atc --model=${project_path}/model/${vanillacnn_prototxt##*/} --weight=${project_path}/model/${vanillacnn_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name2} --soc_version=Ascend310 --input_shape="data:4,3,40,40" --input_format=NCHW"
model_atc3="atc --model=${project_path}/model/${sphereface_prototxt##*/} --weight=${project_path}/model/${sphereface_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name3} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${sphereface_cfg##*/} --input_shape="data:8,3,112,96" --input_format=NCHW"

. ${common_script_dir}/testcase_common.sh

function main() {

    caffe_model=${face_detection_model}
    caffe_prototxt=${face_detection_prototxt}
    model_name=${model_name1}
    model_atc=${model_atc1}
    aipp_cfg=${face_detection_cfg}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    caffe_model=${vanillacnn_model}
    caffe_prototxt=${vanillacnn_prototxt}
    model_name=${model_name2}
    model_atc=${model_atc2}
    aipp_cfg=""
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    
    caffe_model=${sphereface_model}
    caffe_prototxt=${sphereface_prototxt}
    model_name=${model_name3}
    model_atc=${model_atc3}
    aipp_cfg=${sphereface_cfg}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/out
    echo "./
    
    " | bash ${script_path}/run_presenter_server.sh 
    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi
    sleep 2

    mv main ${project_name}
    ./${project_name} &
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    sleep 8

    project_pid=`ps -ef | grep "${project_name}" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

        presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep "${presenter_server_name}" | awk -F ' ' '{print $2}'`
        if [[ ${presenter_server_pid}"X" != "X" ]];then
            echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
            kill -9 ${presenter_server_pid}
            if [ $? -ne 0 ];then
                echo "ERROR: kill presenter server process failed."
                return ${inferenceError}
            fi
        fi
    else 
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    echo "run success"

    return ${success}
}
main
