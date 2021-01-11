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


function DownloadProject () {
        if [  -d "samples" ]; then
        echo "samples is exist"
        #file or folder is empty or not
        DIRECTORY=samples
        if [ "`ls -A $DIRECTORY`" = "" ]; then
        echo "$DIRECTORY is exist but empty"
        cd $HOME 
        git clone https://gitee.com/ascend/samples.git
        else
        echo "$DIRECTORY is exist and not empty"
        echo "please remove the samples folder"
        echo "continue"
        #exit
        fi
       
    else
        echo "samples is not exist"
        cd $HOME 
        git clone https://gitee.com/ascend/samples.git
    fi
}

function DownloadModel () {
     if [  -d "model" ]; then
        DIRECTORY=model
            if [ "`ls -A $DIRECTORY`" = "" ]; then
                    echo "$DIRECTORY is indeed empty"
                    cd model
                    wget https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/models/${project_name}/${model_name}.om
            else
                cd model
                echo "$DIRECTORY is exist and not empty"
                    ls -l|grep -w "$model_name.om"
                    TheModelExist=$?
                    if [ $TheModelExist == 1 ]; then
                        echo "model name is :$model_name.om"
                        echo "but the file is not :$model_name "
                            wget https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/models/${project_name}/${model_name}.om
                    else
                        echo "$DIRECTORY is exist and not empty"
                        echo "continue and do not  download the model again"
                    fi
            fi
    else
        mkdir model
        cd model
        wget https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/models/${project_name}/${model_name}.om
    fi
}


function BuildProject () {
     #build
        echo "BuildProject"
        export DDK_PATH=$HOME/${ACLlib_install_path}/acllib_centos7.6.${TargetTarchitecture}
        export NPU_HOST_LIB=$HOME/${ACLlib_install_path}/acllib_centos7.6.${TargetTarchitecture}/acllib/lib64/stub
        export LANG="C" 
        cd $project_path

        if [  -d "build" ]; then
            echo "build exists"
            cd build
            if [  -d "intermediates" ]; then
                echo "intermediates exists"
            elif [ ! -d "intermediates" ]; then
                echo "intermediates not exists"
                mkdir intermediates
            fi
            cd intermediates
            
        elif [ ! -d "build" ]; then
            echo "build not exists"
            mkdir build
            cd build
            mkdir intermediates
            cd intermediates
        fi
         
        if [  "$TargetTarchitecture" = "aarch64"  ]; then
            echo TargetTarchitecture = $TargetTarchitecture
            $HOME/MindStudio-ubuntu/tools/cmake/bin/cmake $project_path -DCMAKE_SKIP_RPATH=TRUE -Dtarget= -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++-5
            make clean&&make 
        elif [ "$TargetTarchitecture" = "x86_64" ]; then
        echo TargetTarchitecture = $TargetTarchitecture
        $HOME/MindStudio-ubuntu/tools/cmake/bin/cmake $project_path -DCMAKE_SKIP_RPATH=TRUE -Dtarget= -DCMAKE_CXX_COMPILER=g++-7
        make clean&&make 
        fi
        
}

function FindProjectAndThenBuild(){
    for project_name in $app_temp ; do

            cd $HOME/samples
            echo "$project_name"
            model_name=${dic["$project_name"]}
            echo "$model_name"

            #find . -name $project_name|wc -w
            if (( `find . -name $project_name|wc -w` >1 )); then
                echo "there are `find . -name $project_name|wc -w` $project_name"
                echo "please check the files"
                let BuildProjectFailedNumber++
                continue 
            elif (( `find . -name $project_name|wc -w` == 1 )); then
                echo "there are `find . -name $project_name|wc -w` $project_name"
            elif (( `find . -name $project_name|wc -w` < 1 )); then
                echo "there is no $project_name"
                echo "please check the files"
                let BuildProjectFailedNumber++
                continue 
            fi
            
            project_path_temp=`find . -name "$project_name"`
            echo "$project_path_temp"
            project_path=$HOME/samples/$project_path_temp
            echo "$project_path"
            cd $project_path

            DownloadModel
            cd $project_path

            BuildProject  |tee $project_name.log
            # BuildProject  > $project_name.log
            grep  "\[100%\] Built target main" $project_name.log
            TheBuildSuccess=$?
            if [ $TheBuildSuccess == 1 ]; then
                echo ${project_name} build failed.|tee -a $HOME/samples/build.log
                let BuildProjectFailedNumber++
            else
                echo ${project_name} build successfully.|tee -a $HOME/samples/build.log
                let BuildProjectSuccessNumber++
                
            fi
            cd $HOME/samples
        done

}

#main function...
cd $HOME 

DownloadProject

cd $HOME/samples

    ls -l|grep -w "build.log"
    ThebuildlogExist=$?
    if [ $ThebuildlogExist == 0 ]; then
        echo "delete old build.log"
        rm -f build.log
    fi

    BuildProjectSuccessNumber=0
    BuildProjectFailedNumber=0
    if [ "$app_temp" = "all"  ]; then
    app_temp=$app_all_temp
    fi
    FindProjectAndThenBuild
    
    #the next is cppcheck
    #sudo apt  install cppcheck
    #cppcheck --enable=all  $HOME/samples  |tee -a $HOME/samples/cppcheck.log
        echo "cppcheck finished and make the cppcheck.log in $HOME/samples/cppcheck.log"
        echo "$BuildProjectSuccessNumber project build success"|tee -a $HOME/samples/build.log
        echo "$BuildProjectFailedNumber project build failed"|tee -a $HOME/samples/build.log

    if  [ "$run_project" = "yes" ];then
        cd $shellfilepath
        ./copy_files_to_device.sh
    fi