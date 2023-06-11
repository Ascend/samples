#!/bin/bash
export ASCEND_SLOG_PRINT_TO_STDOUT=0
export ASCEND_GLOBAL_LOG_LEVEL=0

CURRENT_DIR=$(
    cd $(dirname ${BASH_SOURCE:-$0})
    pwd
)
cd $CURRENT_DIR

# 导出环境变量
IS_DYNAMIC=$1
if [ ! $ASCEND_HOME_DIR ]; then
    ASCEND_HOME_DIR=/usr/local/Ascend/latest
    source $ASCEND_HOME_DIR/bin/setenv.bash
fi

export DDK_PATH=$ASCEND_HOME_DIR
arch=$(uname -m)
export NPU_HOST_LIB=$ASCEND_HOME_DIR/${arch}-linux/lib64

function main {
    if [[ ${IS_DYNAMIC}"x" = "x" ]]; then
        echo "ERROR: IS_DYNAMIC is invalid!"
        return 1
    fi

    # 1. 生成输入数据和真值数据
    cd $CURRENT_DIR/run/out/test_data/data
    python3 generate_data.py
    if [ $? -ne 0 ]; then
        echo "ERROR: generate input data failed!"
        return 1
    fi
    echo "INFO: generate input data success!"

    # 2. 编译acl可执行文件
    cd $CURRENT_DIR; rm -rf build; mkdir -p build; cd build
    cmake ../src
    if [ $? -ne 0 ]; then
        echo "ERROR: cmake failed!"
        return 1
    fi
    echo "INFO: cmake success!"
    make
    if [ $? -ne 0 ]; then
        echo "ERROR: make failed!"
        return 1
    fi
    echo "INFO: make success!"

    # 3. 运行可执行文件
    cd $CURRENT_DIR/run/out
    if [ $IS_DYNAMIC == 1 ]; then
        echo "INFO: execute dynamic op!"
        ./execute_add_op $IS_DYNAMIC 2048
    else
        echo "INFO: execute static op!"
        ./execute_add_op
    fi
    if [ $? -ne 0 ]; then
        echo "ERROR: acl executable run failed! please check your project!"
        return 1
    fi
    echo "INFO: acl executable run success!"

    # 4. 比较真值文件
    cd $CURRENT_DIR
    python3 $CURRENT_DIR/scripts/verify_result.py       \
        $CURRENT_DIR/run/out/test_data/data/input_0.bin \
        $CURRENT_DIR/run/out/test_data/data/input_1.bin \
        $CURRENT_DIR/run/out/result_files/output_0.bin
    if [ $? -ne 0 ]; then
        echo "ERROR: compare golden data failed! the result is wrong!"
        return 1
    fi
    echo "INFO: compare golden data success!"
}

main
