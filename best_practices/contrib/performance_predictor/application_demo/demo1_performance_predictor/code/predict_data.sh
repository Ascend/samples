#!/bin/bash
dir="output/"
if [ ! -d "$dir" ];then
	mkdir $dir
fi

if [ ! -f "output/RF_model.m" ];then
    echo "文件RF_model.m不存在"
    else
    python3 predict_data.py
fi