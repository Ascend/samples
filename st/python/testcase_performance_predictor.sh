script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
demo1_path=${script_path_temp}/../../best_practices/contrib/performance_predictor/application_demo/demo1_performance_predictor/code
demo2_path=${script_path_temp}/../../best_practices/contrib/performance_predictor/application_demo/demo2_fair_comparison/code

function setEnv() {
  #source /home/HwHiAiUser/.bashrc
  #source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
  source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
  export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  #运行时链接库文件
  export PATH=/usr/local/python3.7.5/bin:$PATH
  export ASCEND_GLOBAL_LOG_LEVEL=1
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest #CANN软件安装后文件存储路径
  export PYTHONPATH=${THIRDPART_PATH}/acllite:$PYTHONPATH
}

function main() {
    setEnv
    # 下载demo1模型
    cd ${demo1_path}
    mkdir output
    if [ ! -f "${demo1_path}/output/RF_model.m" ];then
        wget -O ${demo1_path}/output/RF_model.m https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/performance_predictor/RF_model.m --no-check-certificate
    fi

    # 运行demo1
    bash predict_data.sh
    if [ $? -ne 0 ];then
        return 1
    fi

    # 下载demo2数据
    cd ${demo2_path}
    if [ ! -f "${demo2_path}/data/generated_graphs.json" ];then
        wget -O ${demo2_path}/data/generated_graphs.json https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/performance_predictor/generated_graphs.json --no-check-certificate
    fi

    # 运行demo2
    python3 demo.py
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
