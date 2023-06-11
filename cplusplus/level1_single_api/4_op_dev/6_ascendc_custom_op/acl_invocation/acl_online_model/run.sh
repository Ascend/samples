#!/bin/bash
clear;clear
CURRENT_DIR=$(
    cd $(dirname ${BASH_SOURCE:-$0})
    pwd
); cd $CURRENT_DIR

# 导出环境变量
IS_DYNAMIC=$1
REPLAY_MODE=$2

if [ ! $ASCEND_HOME_DIR ]; then
    export ASCEND_HOME_DIR=/usr/local/Ascend/ascend-toolkit/latest
    source $ASCEND_HOME_DIR/../set_env.sh
fi

PYTHON_VERSION=`python3 -V 2>&1 | awk '{print $2}' | awk -F '.' '{print $1"."$2}'`
export HI_PYTHON=python${PYTHON_VERSION}
export PYTHONPATH=$ASCEND_HOME_DIR/python/site-packages:$PYTHONPATH
export PATH=$ASCEND_HOME_DIR/python/site-packages/bin:$PATH

# 检查当前昇腾芯片的类型
function check_soc_version() {
    SOC_VERSION_CONCAT=`python3 -c '''
import ctypes, os
def get_soc_version():
    max_len = 256
    rtsdll = ctypes.CDLL(f"libruntime.so")
    c_char_t = ctypes.create_string_buffer(b"\xff" * max_len, max_len)
    rtsdll.rtGetSocVersion.restype = ctypes.c_uint64
    rt_error = rtsdll.rtGetSocVersion(c_char_t, ctypes.c_uint32(max_len))
    if rt_error:
        print("rt_error:", rt_error)
        return ""
    soc_full_name = c_char_t.value.decode("utf-8")
    find_str = "Short_SoC_version="
    ascend_home_dir = os.environ.get("ASCEND_HOME_DIR")
    with open(f"{ascend_home_dir}/compiler/data/platform_config/{soc_full_name}.ini", "r") as f:
        for line in f:
            if find_str in line:
                start_index = line.find(find_str)
                result = line[start_index + len(find_str):].strip()
                return "{},{}".format(soc_full_name, result.lower())
    return ""
print(get_soc_version())
    '''`
    if [[ ${SOC_VERSION_CONCAT}"x" = "x" ]]; then
        echo "ERROR: SOC_VERSION_CONCAT is invalid!"
        return 1
    fi
    SOC_FULL_VERSION=`echo $SOC_VERSION_CONCAT | cut -d ',' -f 1`
    SOC_SHORT_VERSION=`echo $SOC_VERSION_CONCAT | cut -d ',' -f 2`
}

function main() {
    if [[ ${IS_DYNAMIC}"x" = "x" ]]; then
        echo "ERROR: IS_DYNAMIC is invalid!"
        return 1
    fi

    if [[ ${REPLAY_MODE}"x" = "x" || ${REPLAY_MODE} = "batch" || ${REPLAY_MODE} = "iterator" ]]; then
        echo "INFO: REPLAY_MODE valid : ${REPLAY_MODE}"
    else
        echo "ERROR: REPLAY_MODE is invalid!"
        return 1
    fi

    # 清除遗留生成文件和日志文件
    rm -rf $HOME/ascend/log/*
    rm -rf $ASCEND_OPP_PATH/vendors/*
    rm -rf custom_op

    # 生成自定义算子工程样例
    JSON_NAME=add_custom
    CAMEL_JSON_NAME=`echo $JSON_NAME | sed -r 's/(^|-|_)(\w)/\U\2/g'`
    msopgen gen -i ../op_dev/${JSON_NAME}.json -f tf -c ai_core-${SOC_SHORT_VERSION} -lan cpp -out ./custom_op
    if [ $? -ne 0 ]; then
        echo "ERROR: msopgen custom op sample failed!"
        return 1
    fi
    echo "INFO: msopgen custom op sample success!"

    cp -rf ../op_dev/* custom_op
    if [ $? -ne 0 ]; then
        echo "ERROR: copy custom op files failed!"
        return 1
    fi
    if [[ $IS_DYNAMIC != 1 ]]; then
        if [[ $REPLAY_MODE = "batch" ]]; then
            sed -i "s/set(BATCH_MODE_REPLAY_LIST/set(BATCH_MODE_REPLAY_LIST ${CAMEL_JSON_NAME}/g" `grep "set(BATCH_MODE_REPLAY_LIST" -rl custom_op/op_kernel/CMakeLists.txt`
        elif [[ $REPLAY_MODE = "iterator" ]]; then
            sed -i "s/set(ITERATOR_MODE_REPLAY_LIST/set(ITERATOR_MODE_REPLAY_LIST ${CAMEL_JSON_NAME}/g" `grep "set(ITERATOR_MODE_REPLAY_LIST" -rl custom_op/op_kernel/CMakeLists.txt`
        fi
    fi
    sed -i "s#/usr/local/Ascend/latest#$ASCEND_HOME_DIR#g" `grep "/usr/local/Ascend/latest" -rl custom_op/CMakePresets.json`

    # 构建自定义算子包并安装
    bash custom_op/run.sh
    if [ $? -ne 0 ]; then
        echo "ERROR: build and install custom op run package failed!"
        return 1
    fi
    echo "INFO: build and install custom op run package success!"

    # 编译acl可执行文件并运行
    bash op_verify/run.sh $IS_DYNAMIC
    if [ $? -ne 0 ]; then
        echo "ERROR: execute acl single op sample failed!"
        return 1
    fi
    echo "INFO: execute acl single op sample success!"
}

check_soc_version
main
