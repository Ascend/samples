#!/bin/bash

cd ../build
./mobilenetv1

if [ $? -ne 0 ];then
    echo "[INFO] The program runs failed"
else
    echo "[INFO] The program runs successfully"
fi
