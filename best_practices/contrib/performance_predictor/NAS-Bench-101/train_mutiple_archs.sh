#!/bin/bash
num=5   #训练架构的数目
arch_file="train_arch" #保存要训练的架构信息的文件名
if [ ! -f "data/generated_graphs.json" ];then
    echo "文件generated_graphs.json不存在"
    python3 generate_graph.py    #生成所有架构
fi
python3 generate_data.py --num ${num} --arch_file ${arch_file}  #随机生成要评估的架构
python3 train_muti.py  --arch_file ${arch_file}