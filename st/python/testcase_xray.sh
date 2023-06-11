script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path_temp}/../../best_practices/contrib
script_path=${project_path}/XRay/scripts

function setEnv() {
  source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
  export CANN_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
  export PATH=/usr/local/python3.7.5/bin:$PATH
}

function main() {
    setEnv
    #设置工具
    cd ${project_path}
    git clone https://github.com/Ascend/tools.git
    cd tools/ais-bench_workload/tool/ais_infer/backend/
    pip3 wheel ./
    pip3 install ./aclruntime-0.0.2-cp37-cp37m-linux_x86_64.whl --user  # aclruntime版本与自己路径下的版本对应

    # 下载数据集
    cd ${project_path}
    mkdir data
    cd data
    if [ ! -f "${project_path}/data/XRay_casting_voc.zip" ];then
        wget -O ${project_path}/data/XRay_casting_voc.zip https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/xray/XRay_casting_voc.zip --no-check-certificate
    fi
    unzip -n -d ${project_path}/data ${project_path}/data/XRay_casting_voc.zip

    cd ${script_path}
    # 转模型
    if [ ! -f "${script_path}/best_model_miou.om" ];then
        wget -O ${script_path}/best_model_miou.onnx https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/xray/best_model_miou.onnx --no-check-certificate
        atc --framework=5 --model=best_model_miou.onnx --output best_model_miou --input_format=NCHW  --log=debug --soc_version=Ascend310
    fi

    # 运行
    sh infer.sh ../../data/XRay_casting_voc ./bin ./best_model_miou.om
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
