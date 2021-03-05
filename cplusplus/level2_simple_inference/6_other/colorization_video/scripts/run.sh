#!/bin/bash
#
#   =======================================================================
#
# Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   1 Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   2 Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   3 Neither the names of the copyright holders nor the names of the
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#   =======================================================================
# ************************Variable*********************************************
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

function SelectCompiler()
{
    TargetMachine=""
    IsUnity=""
    
    # Confirm whether the development environment and operating environment are the same environment
    declare -i CHOICE_TIMES=1
    while [[ ${IsUnity}"X" = "X" ]]
    do
        # three times choice 
        [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
	read -p "Whether the development environment and operating environment are the same? [Y/N](defult:Y):" IsUnity
        if [ ${IsUnity}"z" = "Nz" ] || [ ${IsUnity}"z" = "nz" ]; then
            echo "[INFO] Confirmed that the development environment and operating environment are different!"
            ls /usr/bin |grep expect >/dev/null
            if [[ $? -ne 0 ]];then
                echo "[ERROR] Please execute \e[1;31m sudo apt install expect \e[0m to install the required scripts!"
                return 1
            fi
        elif [ ${IsUnity}"z" = "Yz" ] || [ ${IsUnity}"z" = "yz" ] || [ ${IsUnity}"z" = "z" ]; then
            # In the same environment, the compiler version can be obtained according to uname
            echo "[INFO] Confirmed that the development environment and operating environment are the same environment!"
            ret=`uname -a | grep aarch64`
            if [[ $? -ne 0 ]];then
                TargetMachine="ASIC"
            else
                TargetMachine="ATLAS"
            fi
            return 0
        else
            echo "[ERROR] Please input Y/N!"
            IsUnity=""
        fi
    done
    if [[ ${CHOICE_TIMES} -gt 3 ]];then
        echo "[ERROR] Error input more than three times, the program terminates!"
        return 1
    fi
    
    # Separate the scene, the user needs to give the target environment compiler type
    declare -i CHOICE_TIMES=1
    while [[ ${TargetMachine}"X" = "X" ]]
    do
        # three times choice 
        [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
        echo "1.ATLAS"
        echo "2.ASIC"
        read -p "Please select operating equipment(defult:1):" TargetMachine
        
        if [[ ${TargetMachine}"X" = "1X" ]] || [[ ${TargetMachine}"X" = "ATLASX" ]] || [[ ${TargetMachine}"X" = "atlasX" ]] || [[ ${TargetMachine}"X" = "X" ]] || [[ ${TargetMachine}"X" = "1.ATLASX" ]] || [[ ${TargetMachine}"X" = "1.atlasX" ]]; then
            TargetMachine="ATLAS"
        elif [[ ${TargetMachine}"X" = "2X" ]] || [[ ${TargetMachine}"X" = "ASICX" ]] || [[${TargetMachine}"X" = "asicX"]] || [[ ${TargetMachine}"X" = "2.ASICX" ]] || [[ ${TargetMachine}"X" = "2.asicX" ]]; then
            TargetMachine="ASIC"
        else
            echo "[ERROR] Please enter the operating device, the content is 1.ATLAS or 2.ASIC!"
            TargetMachine=""
        fi
    done
    if [[ ${CHOICE_TIMES} -gt 3 ]];then
        echo "[ERROR] Error input more than three times, the program terminates!"
        return 1
    fi
    
    return 0
}

function ModelConvert()
{
    # Determine whether there is a better om model
    ret=`find ${ModelPath} -maxdepth 1 -name "colorization.om" 2> /dev/null`
    if [[ ${ret} ]];then
        declare -i CHOICE_TIMES=1
        while [[ ${response}"X" = "X" ]]
        do
            # three times choice 
            [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
	    read -p "The colorization.om already exists. Do you want to keep it? [Y/N](defult:y):" response
            if [ ${response}"z" = "Yz" ] || [ ${response}"z" = "yz" ] || [ ${response}"z" = "z" ]; then
                echo "[INFO] Existing model, skip model conversion!"
                return 0
            elif [ ${response}"z" = "Nz" ] || [ ${response}"z" = "nz" ]; then
                echo "Existing model, re-transform the model"
            else
                echo "[ERROR] Please input Y/N!"
                response=""
            fi
        done
        if [[ ${CHOICE_TIMES} -gt 3 ]];then
            echo "[ERROR] Error input more than three times, the program terminates!"
            return 1
        fi
    fi
    
    # Download the original model file and test data
    wget -O ${ModelPath}/colorization.prototxt https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.prototxt --no-check-certificate
    wget -O ${ModelPath}/colorization.caffemodel https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/colorization/colorization.caffemodel --no-check-certificate
    wget -O ${ModelPath}/../data/black-white_video.mp4 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization_video/black-white_video.mp4 --no-check-certificate

    # Set environment variables and perform model conversion
    export LD_LIBRARY_PATH=${install_path}/atc/lib64
    atc --input_shape="data_l:1,1,224,224" --weight="${ModelPath}/colorization.caffemodel" --input_format=NCHW --output="${ModelPath}/colorization" --soc_version=Ascend310 --framework=0 --model="${ModelPath}/colorization.prototxt" --log=info
    if [ $? -ne 0 ];then
        echo "[ERROR] convert model failed, Please check your environment!"
        return 1
    else
        echo "[INFO] Model convert success!"
    fi
    
    # Delete process file
    rm ${ModelPath}/*.caffemodel
    rm ${ModelPath}/*.prototxt
    rm -rf ${ModelPath}/kernel_meta
    return 0
}

function BuildSample()
{

    if [[ ${TargetMachine}"X" = "ASICX" ]];then
        Targetcompiler="g++"
	Targetkernel="x86"
        export DDK_PATH=${HOME}/Ascend/ascend-toolkit/latest
    else
        Targetcompiler="aarch64-linux-gnu-g++"
	Targetkernel="arm"
        export DDK_PATH=${HOME}/Ascend/ascend-toolkit/latest/arm64-linux
    fi

    # Check if it has been compiled
    ret=`find ${ScriptPath}/.. -name "main" 2> /dev/null`
    if [ -d ${ScriptPath}/../build/intermediates/host ] && [ ${ret} ];then
        declare -i CHOICE_TIMES=1
        while [[ ${recompiled}"X" = "X" ]]
        do
            # three times choice 
            [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
	    read -p "The program has been compiled. Do you want to keep it? [Y/N](defult:y):" recompiled
            if [ ${recompiled}"z" = "Yz" ] || [ ${recompiled}"z" = "yz" ] || [ ${recompiled}"z" = "z" ]; then
                echo "[INFO] Skip compilation!"
                return 0
            elif [ ${recompiled}"z" = "Nz" ] || [ ${recompiled}"z" = "nz" ]; then
                echo "Compiled, recompile the program!"
            else
                echo "[ERROR] Please input Y/N!"
                recompiled=""
            fi
        done
        if [[ ${CHOICE_TIMES} -gt 3 ]];then
            echo "[ERROR] Error input more than three times, the program terminates!"
            return 1
        fi
    fi
    
    # Preparation for compilation
    if [[ ${recompiled} ]];then
        rm -rf ${ScriptPath}/../build/intermediates/host
    fi
    mkdir -p ${ScriptPath}/../build/intermediates/host
    cd ${ScriptPath}/../build/intermediates/host
    
    echo ${Targetcompiler}

    # Start compiling
    export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    cmake ../../../src -DCMAKE_CXX_COMPILER=${Targetcompiler} -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "[ERROR] cmake error, Please check your environment!"
        return 1
    fi
    make
    if [ $? -ne 0 ];then
        echo "[ERROR] build failed, Please check your environment!"
        return 1
    fi
    cd -
}

function running()
{
    # Enter the operating environment ip and password to facilitate subsequent operations
    if [ ${IsUnity}"z" = "Nz" ] || [ ${IsUnity}"z" = "nz" ];then
        declare -i CHOICE_TIMES=1
        while [[ ${operating_ip}"X" = "X" ]] || [[ ${operating_passwd}"X" = "X" ]] || [[ ${operating_user}"X" = "X" ]]
        do
            # three times choice 
            [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
            read -p "Please enter the operating user (defult: HwHiAiUser):" operating_user
            read -p "Please enter the operating environment ip (defult: 192.168.1.2):" operating_ip
            read -p "Please enter the operating environment login password (defult: Mind@123):" operating_passwd
            if [ ${operating_user}"z" = "z" ]; then
                operating_user="HwHiAiUser"
            fi
            if [ ${operating_ip}"z" = "z" ]; then
                operating_ip="192.168.1.2"
            fi
            if [ ${operating_passwd}"z" = "z" ]; then
                operating_passwd="Mind@123"
            fi    
            if [ ${operating_ip} ] && [ ${operating_passwd} ] && [ ${operating_user} ]; then
                echo $operating_user
                expect  << EOF
                set operating_user ${operating_user}
                set operating_ip ${operating_ip}
                set operating_passwd ${operating_passwd}
                set timeout 4
                spawn ssh \${operating_user}@\${operating_ip} "exit"
                expect {
                    -re "password" {send "\${operating_passwd}\r";exp_continue}
                    -re "yes/no" {send "yes\r";exp_continue}
                }
                catch wait result 
                if { [lindex \$result 3] != 0 } {
                    exit [lindex \$result 3] 
                } 
EOF

                if [ $? -ne 0 ];then
                    echo "[ERROR] environment user, ip or password input error, please check!"
                    operating_user=""
                    operating_ip=""
                    operating_passwd=""
                fi
            else
                echo "[ERROR] Please input environment ip and environment login password!"
                operating_user=""
                operating_ip=""
                operating_passwd=""
            fi
        done
        if [[ ${CHOICE_TIMES} -gt 3 ]];then
            echo "[ERROR] Error input more than three times, the program terminates!"
            return 1
        fi

        # presnetserver ip check
        if [[ ${TargetMachine}"X" = "ASICX" ]];then
            declare -i CHOICE_TIMES=1
            while [[ ${operating_eth0ip}"X" = "X" ]]
            do
                # three times choice 
                [[ ${CHOICE_TIMES} -gt 3 ]] && break || ((CHOICE_TIMES++))
                read -p "Please enter the internal network ip of ai1s environment eth0 ip(eg:192.168.1.162):" operating_eth0ip
                if [ ${operating_eth0ip} ]; then
                    expect  << EOF
                    set operating_user ${operating_user}
                    set operating_ip ${operating_ip}
                    set operating_passwd ${operating_passwd}
                    set operating_eth0ip ${operating_eth0ip}
                    set timeout 4
                    spawn ssh \${operating_user}@\${operating_ip} "ifconfig eth0| grep \${operating_eth0ip};exit"
                    expect {
                        -re "password" {send "\${operating_passwd}\r";exp_continue}
                        -re "yes/no" {send "yes\r";exp_continue}
                    }
                    catch wait result 
                    if { [lindex \$result 3] != 0 } {
                        exit [lindex \$result 3] 
                    } 
EOF
                    if [ $? -ne 0 ];then
                        echo "[ERROR] ai1s environment eth0 ip input error, Please check!"
                        operating_eth0ip=""
                    fi
                else
                    echo "[ERROR] Please input ai1s environment eth0 ip!"
                    operating_eth0ip=""
                fi
            done
            if [[ ${CHOICE_TIMES} -gt 3 ]];then
                echo "[ERROR] Error input more than three times, the program terminates!"
                return 1
            fi
        else
           operating_eth0ip=${operating_ip}
        fi
        sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${operating_eth0ip}/g" ${ScriptPath}/colorization.conf
        sed -i "s/presenter_view_ip=[0-9.]*/presenter_view_ip=${operating_eth0ip}/g" ${ScriptPath}/colorization.conf
        cp -f ${ScriptPath}/colorization.conf ${ScriptPath}/../out

        # Transfer the file to the running device and clear the log to start running the program
        expect  << EOF
        set operating_user ${operating_user}
        set operating_ip ${operating_ip}
        set operating_passwd ${operating_passwd}
        set ScriptPath ${ScriptPath}
        set Targetkernel ${Targetkernel}
        set timeout -1
        spawn scp -r \${ScriptPath}/../../colorization_video \${operating_user}@\${operating_ip}:/home/\${operating_user} 
        expect {
            -re "yes/no" { send "yes\r";exp_continue } 
            -re "password" { send "\${operating_passwd}\r";exp_continue }
        }
        catch wait result  
        if { [lindex \$result 3] != 0 } {
            exit [lindex \$result 3] 
        }
        spawn scp -r \${ScriptPath}/../../../../../common \${operating_user}@\${operating_ip}:/home/\${operating_user} 
        expect {
            -re "yes/no" { send "yes\r";exp_continue } 
            -re "password" { send "\${operating_passwd}\r";exp_continue }
        }
        catch wait result  
        if { [lindex \$result 3] != 0 } {
            exit [lindex \$result 3] 
        }
        spawn ssh \${operating_user}@\${operating_ip} "cd /home/\${operating_user}/common;bash run_presenter_server.sh /home/\${operating_user}/colorization_video/scripts/colorization.conf;sleep 3;export LD_LIBRARY_PATH=/home/\${operating_user}/ascend_ddk/\${Targetkernel}/lib:/home/\${operating_user}/Ascend/acllib/lib64;rm -rf /var/log/npu/slog/host-0/* ;cd /home/\${operating_user}/colorization_video/out;./main ../data/black-white_video.mp4 &"
        expect {
            -re "yes/no" { send "yes\r";exp_continue } 
            -re "password" { send "\${operating_passwd}\r";exp_continue }
            -re "The sample starts to run" { exit 0 }
            -re "failed" { exit 1 }
        }
EOF
        if [ $? -ne 0 ];then
            # The program fails to run, and the log is returned
            expect  << EOF
            set operating_user ${operating_user}
            set operating_ip ${operating_ip}
            set operating_passwd ${operating_passwd}
            set ScriptPath ${ScriptPath}
            set timeout -1
            spawn ssh \${operating_user}@\${operating_ip} "bash /home/\${operating_user}/colorization_video/scripts/kill_run.sh"
            expect {
                -re "yes/no" { send "yes\r";exp_continue } 
                -re "password" { send "\${operating_passwd}\r";exp_continue }
            }
            catch wait result 
            if { [lindex \$result 3] != 0 } {
                exit [lindex \$result 3] 
            }
            spawn scp -r \${operating_user}@\${operating_ip}:/var/log/npu/slog/host-0 \${ScriptPath}
            expect {
                -re "yes/no" { send "yes\r";exp_continue } 
                -re "password" { send "\${operating_passwd}\r";exp_continue }
            }
            catch wait result 
            if { [lindex \$result 3] != 0 } {
                exit [lindex \$result 3] 
            } 
EOF
            if [ $? -ne 0 ];then
                echo "[ERROR] The sample failed to run, and the log or process return failed, please check the environment!"
                return 1
            else
                echo "[ERROR] The sample failed to run, please check the ${ScriptPath}/host_0 log, or log in to the running device to check the environment!"
                return 1
            fi
        else
            echo "[INFO] The sample runs successfully, Please visit http://${operating_ip}:7007 for display server!"
            read -p "Enter any command to stop the application:" operating_eth0ip
            expect  << EOF
            set operating_user ${operating_user}
            set operating_ip ${operating_ip}
            set operating_passwd ${operating_passwd}
            set ScriptPath ${ScriptPath}
            set timeout -1
            spawn ssh \${operating_user}@\${operating_ip} "bash /home/\${operating_user}/colorization_video/scripts/kill_run.sh"
            expect {
                -re "yes/no" { send "yes\r";exp_continue } 
                -re "password" { send "\${operating_passwd}\r";exp_continue }
            }
            catch wait result 
            if { [lindex \$result 3] != 0 } {
                exit [lindex \$result 3] 
            } 
EOF
        fi
    else
        # Run the program when the development environment and the operating environment are the same
        if [[ ${TargetMachine}"X" = "ASICX" ]];then
            operating_eth0ip=`ifconfig eth0|grep "inet " | awk -F ' ' '{print $2}'`
            ifconfig eth0|grep ${operating_eth0ip} > /dev/null
            if [ $? -ne 0 ];then
                echo "[ERROR] eth0 ip error,Please check ip"
                return 1
            fi
        else
            operating_eth0ip=${operating_ip}
        fi
        sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${operating_eth0ip}/g" ${ScriptPath}/colorization.conf
        sed -i "s/presenter_view_ip=[0-9.]*/presenter_view_ip=${operating_eth0ip}/g" ${ScriptPath}/colorization.conf
	cp -f ${ScriptPath}/colorization.conf ${ScriptPath}/../out
        cd ${ScriptPath}/../../../../../common
        bash run_presenter_server.sh ${ScriptPath}/colorization.conf > /dev/null
        if [ $? -ne 0 ];then
            echo "[ERROR] The program failed to run, please check the environmoent!"
	    cd ${ScriptPath}
	    bash kill_run.sh
            return 1
        fi
        export LD_LIBRARY_PATH=${HOME}/ascend_ddk/${Targetkernel}/lib:${HOME}/Ascend/acllib/lib64:$LD_LIBRARY_PATH
        cd ${ScriptPath}/../out
        ./main ../data/black-white_video.mp4 &
	sleep 1
        if [ $? -ne 0 ];then
            echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
	    cd ${ScriptPath}
            bash kill_run.sh
            return 1
        else
            echo "[INFO] The program runs successfully, Please visit http://Public network ip:7007 for display server!"
            read -p "Enter any command to stop the application:" operating_eth0ip
	    cd ${ScriptPath}
            bash kill_run.sh 
	    return 0
        fi
    fi
}

# ********************** run sample ************************************
function main()
{
    echo "[INFO] The colorization sample starts running"
    # Determine the compiler of the target machine
    SelectCompiler
    if [ $? -ne 0 ];then
        return 1
    fi

    # Model Convert
    ModelConvert
    if [ $? -ne 0 ];then
        return 1
    fi
    
    # Build sample
    BuildSample
    if [ $? -ne 0 ];then
        return 1
    fi

    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
