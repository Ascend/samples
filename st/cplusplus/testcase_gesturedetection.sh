caffe_model1="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_iter_440000.caffemodel"
caffe_prototxt1="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_deploy.prototxt"
aipp_cfg="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/insert_op.cfg"
pb1="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/stgcn_fps30_sta_ho_ki4.pb"
model_name_1="pose_deploy"
model_name_2="stgcn_fps30_sta_ho_ki4"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/gesturedetection/test_image/"
project_name="cplusplus_Gesturedetection"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/contrib/gesturedetection/scripts
project_path=${script_path}/..
ModelPath=${project_path}/model
common_script_dir=${script_path}/../../../../common/
run_command="./main"
jpg=".jpg"
model_atc1="atc --input_shape="data:1,3,128,128" --weight=${project_path}/model/${caffe_model1##*/} --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name_1} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --framework=0 --model=${project_path}/model/${caffe_prototxt1##*/}"
model_atc2="atc --input_shape="input_features:1,2,30,14" --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name_2} --soc_version=Ascend310 --framework=3 --model=${project_path}/model/${pb1##*/}"
. ${common_script_dir}/testcase_common.sh
function main() {
    # 下载测试集
    for ((i=0; i<50; i ++))
    do
    if [ ! -f "${ModelPath}/../data/${i}${jpg}" ];then
    	wget -O ${project_path}/data/$i".jpg"  ${data_source}$i".jpg"  --no-check-certificate
    fi
    if [ $? -ne 0 ];then
            echo "Download jpg failed, please check network."
            return 1
        fi
    done
    # 转模型
    tf_model=${pb1}
    model_name=${model_name_2}
    model_atc=${model_atc2}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    tf_model=" "
    caffe_model=${caffe_model1}
    caffe_prototxt=${caffe_prototxt1}
    model_name=${model_name_1}
    model_atc=${model_atc1}
    modelconvert
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main

