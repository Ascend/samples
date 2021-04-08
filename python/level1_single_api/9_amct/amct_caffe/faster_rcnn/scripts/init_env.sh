#!/bin/bash
set -e
if [[ $# -lt 1 || $1 == 'h' || $1 == 'help' ]]
then
    echo "help info:"
    echo "  usage bash init_env.sh [CPU/GPU] [caffe dir] [(optional)python3 verison] [(optional)python3 python3m dir]"
    echo "   arg[1](required): specify run mode CPU or GPU"
    echo "   arg[2](required): your caffe-master dir"
    echo "   arg[3](optional): python3 version, default: python3, other like: python3.7"
    echo "   arg[4](optional): specify python3m dir, default: /usr/include/python3.7m"
    echo "   arg[5](optional): specify 'with_benchmark' to do benchmark test"
    exit 0
fi

if [ $# -lt 2 ]
then
    echo "Must specify GPU/CPU mode and caffe_dir"
    echo ""
    echo "use \"bash init_env.sh h\"or \"bash init_env.sh help\" for help info"
    echo "usage: bash init_env.sh [CPU/GPU] [caffe dir] [(optional)python3 verison] [(optional)python3 python3m dir] [(optional) with_benchmark]"
    echo "e.g. bash init_env.sh GPU your_caffe_master_dir"
    exit -1
fi

if [ $1 == "GPU" ]
then
    RUN_MODE=GPU
elif [ $1 == "CPU" ]
then
    RUN_MODE=CPU
else
    echo "Only support CPU or GPU mode, ${1} not supported"
    exit -1
fi

CAFFE_DIR=$2
echo "caffe_dir is: ${CAFFE_DIR}"

PYTHON3_V=python3
if [ $# -ge 4 ]
then
    PYTHON3_V=$3
fi

PYTHON3_INCLUDE_DIR=/usr/include/python3.7m
if [ $# -ge 4 ]
then
    PYTHON3_INCLUDE_DIR=$4
fi
BENCHMARK_FLAG=without_benchmark
if [[ $# -eq 3 && $3 == 'with_benchmark' ]]; then
    BENCHMARK_FLAG=with_benchmark
elif [[ $# -eq 5 && $5 == 'with_benchmark' ]]; then
    BENCHMARK_FLAG=with_benchmark
fi

bash download_and_assemble.sh ${BENCHMARK_FLAG}
bash modified_files.sh ${RUN_MODE} ${CAFFE_DIR} ${PYTHON3_V} ${PYTHON3_INCLUDE_DIR} ${BENCHMARK_FLAG}
cd ../src/python_tools && make
