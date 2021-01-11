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


cd $HOME

        if [  -d "project_run_test_temp" ]; then
        echo "project_run_test_temp is exist"
        DIRECTORY=project_run_test_temp
        if [ "`ls -A $DIRECTORY`" = "" ]; then
        echo "$DIRECTORY is exist but empty"
        else
        echo "$DIRECTORY is exist and not empty"
        echo "remove the project_run_test_temp folder"
		rm -rf project_run_test_temp
        echo "then mkdir project_run_test_temp"
		mkdir project_run_test_temp
        #exit
        fi
       
    else
        echo "project_run_test_temp is not exist"
        cd $HOME 
        mkdir project_run_test_temp
    fi


if [ "$app_temp" = "all"  ]; then
    app_temp=$app_all_temp
fi

for project_name in $app_temp ; do

	cd $HOME/samples
	echo "$project_name"
	model_name=${dic["$project_name"]}
	echo "$model_name"

	project_path_temp=`find . -name "$project_name"`
	echo "$project_path_temp"
	project_path=$HOME/samples/$project_path_temp
	echo "$project_path"
	cd $project_path

	mkdir $HOME/project_run_test_temp/${project_name}
	cp -r $project_path/data $HOME/project_run_test_temp/${project_name}
	cp -r $project_path/model $HOME/project_run_test_temp/${project_name}
	cp -r $project_path/src $HOME/project_run_test_temp/${project_name}
	cp -r $project_path/out $HOME/project_run_test_temp/${project_name}
    
	cd $HOME/samples
done

cp $shellfilepath/dev_file_build.config $shellfilepath/run_in_device

scp -r $HOME/project_run_test_temp HwHiAiUser@${IP_address}:
scp -r $shellfilepath/run_in_device HwHiAiUser@${IP_address}:
