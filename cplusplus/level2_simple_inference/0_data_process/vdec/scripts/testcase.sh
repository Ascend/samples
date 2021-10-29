data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdec"
verify_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdec/verify_image"
project_name="cplusplus_vdec"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

common_script_dir=${script_path}/../../../../../common/
run_command="./main "

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

. ${common_script_dir}/testcase_common.sh

function main() {

    # 下载测试集和验证集
    wget -O ${project_path}/data/"vdec_h265_1frame_rabbit_1280x720.h265"  ${data_source}/"vdec_h265_1frame_rabbit_1280x720.h265" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download vdec_h265_1frame_rabbit_1280x720.h265 failed, please check Network."
        return 1
    fi
    mkdir -p ${project_path}/verify_image/
    wget -O ${project_path}/verify_image/"image1.yuv"  ${verify_source}/"image1.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image1.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image2.yuv"  ${verify_source}/"image2.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image2.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image3.yuv"  ${verify_source}/"image3.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image3.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image4.yuv"  ${verify_source}/"image4.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image4.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image5.yuv"  ${verify_source}/"image5.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image5.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image6.yuv"  ${verify_source}/"image6.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image6.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image7.yuv"  ${verify_source}/"image7.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image7.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image8.yuv"  ${verify_source}/"image8.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image8.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image9.yuv"  ${verify_source}/"image9.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image9.yuv failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/verify_image/"image10.yuv"  ${verify_source}/"image10.yuv" --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download image10.yuv failed, please check Network."
        return 1
    fi


    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    run_picture
    if [ $? -eq ${inferenceError} ];then
        return ${inferenceError}
    elif [ $? -eq ${verifyResError} ];then
	return ${verifyResError}
    fi

    return ${success}
}
main

