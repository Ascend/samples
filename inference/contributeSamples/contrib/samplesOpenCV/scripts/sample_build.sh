#!/bin/bash

if [ -d ../build ];then
    rm -rf ../build
fi
mkdir -p ../build && cd ../build

cmake ../src
if [ $? -ne 0 ];then
    echo "[ERROR] cmake error, Please check your environment!"
fi

cmake --build . -j8
if [ $? -ne 0 ];then
    echo "[ERROR] build failed, Please check your environment!"
fi
