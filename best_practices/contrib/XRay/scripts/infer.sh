#!/bin/bash

dataset_dir=$1
save_path=$2
model=$3

if [ ! -d "new_result" ];then
    mkdir new_result
    else
    rm -rf new_result
    mkdir new_result
fi

python3 preprocess.py --dataset_dir ${dataset_dir} --save_path ${save_path}
echo "preprocess done"
python3 ../../tools/ais-bench_workload/tool/ais_infer/ais_infer.py --model ${model} --input ${save_path} --batchsize 1 --output new_result/
echo "ais_infer done"

mask=${save_path}"_mask"
# echo $mask

cd new_result
mv ../new_result/*/ bs1/
cd ..

python3 postprocess.py --resdir new_result/bs1 --labeldir ${mask}
echo "postprocess done"