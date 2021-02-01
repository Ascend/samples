#!/bin/bash
path_cur=$(cd `dirname $0`; pwd)
build_type="Release"

function preparePath() {
    mkdir -p  $1
    cd  $1
}

function buildA300() {
    echo ${path_cur}
    path_build=$path_cur/build
    preparePath $path_build
    CC=gcc CXX=g++ cmake -DCMAKE_BUILD_TYPE=$build_type ..
    make -j
    if [ $? -ne 0 ]; then
        echo "Build Failed"
        exit -1
    fi
    cd ..
}

paramNum=$#
params=$@
export ASCEND_VERSION=ascend-toolkit/latest
export ARCH_PATTERN=acllib
buildA300

# copy config file and model into dist
mkdir ./dist/Data
cp -r ./Data ./dist/

ls ./dist/Data/Models/TextDetection/*.om >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "[Warning] No .om file in ./dist/Data/Models/TextDetection, please convert the model to .om file first."
fi

ls ./dist/Data/Models/TextRecognition/*.om >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "[Warning] No .om file in ./dist/Data/Models/TextRecognition, please convert the model to .om file first."
fi


