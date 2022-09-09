tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/3D_gesture_recognition/3d_gesture_recognition.pb"
model_name="3d_gesture_recognition"

data_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/3D_gesture_recognition/testdata/"
data_name="test_float32_actiontype7.bin"
verify_source="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/3D_gesture_recognition/verifytxt/"
verify_name="test_float32_actiontype7.txt"
project_name="3d_gesture_recognition"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../python/contrib/3Dgesture_recognition
common_script_dir=${project_path}/../../../common/
run_command="python3.6 3Dgesture_recognition.py ../data"
model_atc="atc  --input_shape="X:1,16,112,112,3"  --input_format=NDHWC --model=${project_path}/model/${tf_model##*/}  --output=${HOME}/models/${project_name}/${model_name} --framework=3 --soc_version=Ascend310"

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

    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi
    verify_data=`cat ${project_path}/verify_image/test_float32_actiontype7.txt`
    result_data=`cat ${project_path}/out/test_float32_actiontype7.txt`
    if [ "$verify_data"x != "$result_data"x ];then
        echo "ERROR: The result  is wrong!"
        return ${verifyResError}
    else
        return ${success}     
    fi

}
main
