#!/bin/bash

app_temp=`sed '/^project=/!d;s/.*=//' dev_file_build.config`  
ACLlib_install_path=`sed '/^ACLlib_install_path=/!d;s/.*=//' dev_file_build.config`  
IP_address=`sed '/^IP_address=/!d;s/.*=//' dev_file_build.config`  
TargetTarchitecture=`sed '/^TargetTarchitecture=/!d;s/.*=//' dev_file_build.config`  
app_all_temp=`sed '/^build_all_project=/!d;s/.*=//' dev_file_build.config`  
run_project=`sed '/^run_project_yes_or_not=/!d;s/.*=//' dev_file_build.config`  

shellfilepath=`cd $(dirname $0); pwd -P`
echo $shellfilepath

declare -A dic
dic=([venc]="model1" [vdec]="model2" [gpio]="model3" [i2c]="model4" [crop]="model5" \
[googlenet_imagenet_multi_batch]="googlenet_multibatch" [googlenet_imagenet_picture]="googlenet")


test_path="/home/HwHiAiUser/project_run_test_temp"
mkdir $test_path/run_output_temp

cd $test_path
echo $test_path

if [ "$app_temp" = "all"  ]; then
app_temp=$app_all_temp
fi

for project_name in $app_temp ; do

    echo "$project_name"
    model_name=${dic["$project_name"]}
    echo "$model_name"

    mkdir $test_path/$project_name/out/output
    cd $test_path/$project_name/out
    ./main ../data |tee ./output/$project_name.log
    mkdir -p $test_path/run_output_temp/$project_name/output
    cp -r $test_path/$project_name/out/output/. $test_path/run_output_temp/$project_name/output

done
