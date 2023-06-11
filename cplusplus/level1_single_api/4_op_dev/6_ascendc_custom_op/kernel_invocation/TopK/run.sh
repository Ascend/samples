#!/bin/bash
clear;clear
# 清除之前遗留的文件
rm -rf *.vcd *.dump *.log *.bin *.o *pu build output/*.bin input/*.bin
# 不需要TIK打印出内存信息
export PRINT_TIK_MEM_ACCESS=FALSE

# 获取当前的目录
CURRENT_DIR=$(
    cd $(dirname ${BASH_SOURCE:-$0})
    pwd
); cd $CURRENT_DIR

declare -A VersionMap
VersionMap["ascend910"]="Ascend910A"
VersionMap["ascend310p"]="Ascend310P1"

# 指向昇腾软件包安装地址，导出环境变量
if [ ! $ASCEND_HOME_DIR ]; then
    ASCEND_HOME_DIR=/usr/local/Ascend/ascend-toolkit/latest
    source $ASCEND_HOME_DIR/../set_env.sh
fi

# 指定当前sample的算子文件名
FILE_NAME=$1

# 指定芯片版本: ascend910, ascend310p
SOC_VERSION=$2
if [ ${SOC_VERSION}"x" = "x" ]; then
    echo "ERROR: SOC_VERSION is not specified! please specify ascend910 or ascend310p!"
    exit -1
fi
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ASCEND_HOME_DIR/tools/tikicpulib/lib/${VersionMap[$SOC_VERSION]}:$ASCEND_HOME_DIR/toolkit/tools/simulator/${VersionMap[$SOC_VERSION]}/lib

# 指定运行的核: AiCore, VectorCore
CORE_TYPE=$3
if [ ${CORE_TYPE}"x" = "x" ]; then
    echo "WARNING: CORE_TYPE is not specified, using AiCore as default."
    CORE_TYPE=AiCore
fi

# 指定运行模式: cpu, npu
RUN_MODE=$4
if [ ${RUN_MODE}"x" = "x" ]; then
    echo "WARNING: RUN_MODE is not specified, using cpu as default."
    RUN_MODE=cpu
fi

# 生成计算输入数据和对比用的真值数据
python3 $FILE_NAME.py

function compile_and_execute() {
    # 使用cmake编译cpu侧或者npu侧算子
    mkdir -p build;cd build;               \
    cmake ..                               \
        -Dsmoke_testcase=$1                \
        -Dproduct_type=$2                  \
        -Dcore_type=$3                     \
        -Dinstall_path=$ASCEND_HOME_DIR && \
    make ${1}_${4} VERBOSE=1 &&            \
    cd -

    if [ $? -ne 0 ]; then
        echo "ERROR: compile op on failed!"
        return 1
    fi
    echo "INFO: compile op on ${RUN_MODE} succeed!"

    # 执行生成的可执行文件
    ./${1}_${4}
    if [ $? -ne 0 ]; then
        echo "ERROR: execute op on ${RUN_MODE} failed!"
        return 1
    fi
    echo "INFO: execute op on ${RUN_MODE} succeed!"
}
compile_and_execute $FILE_NAME $SOC_VERSION $CORE_TYPE $RUN_MODE

# 验证计算结果
echo "md5sum: ";md5sum output/*.bin
