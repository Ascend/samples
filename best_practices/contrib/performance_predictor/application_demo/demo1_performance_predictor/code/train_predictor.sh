#!/bin/bash
dir="output/"
if [ ! -d "$dir" ];then
	mkdir $dir
fi

if [ ! -f "data/generated_graphs.json" ];then
    echo "文件generated_graphs.json不存在，正在生成该文件。"
    python3 generate_graph.py    #生成所有架构
fi

if [ ! -f "data/data.json" ];then
    echo "文件data.json不存在"
    else
    python3 train_predictor.py
fi