English|[中文](README_200DK_CN.md)
# Basic Environment Configuration  
This readme file describes how to configure the basic environment, including the sudo permission, apt source, Atlas 200 DK networking, and environment variables. If they have been configured, skip this workflow.  

 **Run the following commands as a common user in the development environment. The following takes **HwHiAiUser** as an example. Replace it with the actual running user.** 


1. Grant the sudo permission to the **HwHiAiUser** user.


   Switch to the **root** user.  
    **su root** 

   Grant the write permission on the **sudoers** file and open the file.  
    **chmod u+w /etc/sudoers**   
    **vi /etc/sudoers** 

   Add the following content below **`# User privilege specification`** in the **sudoers** file.  
    **HwHiAiUser ALL=(ALL:ALL) ALL** 

   ![](https://images.gitee.com/uploads/images/2020/1128/144046_7c02d0d0_7401379.png "Screenshot.png")

   Remove the write permission on the **/etc/sudoers** file.  
    **chmod u-w /etc/sudoers**    
   Switch to the common user.  
    **exit**  

2. Configure the apt sources.  

    Configure the Tsinghua apt sources of Ubuntu 18.04 (x86).

    **sudo vi /etc/apt/sources.list** 

    Replace the source file content with the following Tsinghua apt sources.

    ```
    # By default, the source code images are commented out to speed up the apt update. You can uncomment them as required.
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
    ```
    Update the sources.   
    **sudo apt-get update** 
    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >  **If the source fails to be updated, you can try a new source available at https://www.cnblogs.com/dream4567/p/9690850.html**. 


3. Install a compiler in the development environment. 

     **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu**   

4. Add the following environment variables to the development environment for ATC model conversion. 

    1. Open the **.bashrc** file.  
        **vim ~/.bashrc** 

        Add the following environment variables to the file.  
        **export install_path=\$HOME/Ascend/ascend-toolkit/latest**

        **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

        **export ASCEND_OPP_PATH=\${install_path}/opp**  

        **export LD_LIBRARY_PATH=\${install_path}/atc/lib64** 

        The environment variable **PYTHONPATH** is incompatible with CANN 20.0 and 20.1. Run the corresponding command to add **PYTHONPATH** based on the CANN version.

        - For CANN 20.0

            **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**  

        - For CANN 20.1

            **export PYTHONPATH=\\${install_path}/atc/python/site-packages:\\${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:\\${install_path}/atc/python/site-packages/schedule_search.egg:\\$PYTHONPATH**  

        Save the configuration and exit.  
        **wq!**
        >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
        >**- Replace install_path with the actual installation path.**  
        >**- If the development environment and operating environment are set up on the same server, do not configure LD_LIBRARY_PATH, avoiding the conflict with LD_LIBRARY_PATH in the operating environment when running the sample.**

    2. Make the configuration take effect.   
        **source ~/.bashrc**  

 **Perform the following operations in the operating environment (Atlas 200 DK).**  
1. Log in to the operating environment.  
    ssh HwHiAiUser@X.X.X.X  

2. Grant the sudo permission to the **HwHiAiUser** user.


    Switch to the **root** user. The default password of the **root** user is Mind@123.   
    **su root**

    Grant the write permission on the **sudoers** file and open the file.  
    **chmod u+w /etc/sudoers**   
    **vi /etc/sudoers** 

    Add the following content below **`# User privilege specification`** in the **sudoers** file.  
     **HwHiAiUser    ALL=(ALL:ALL) ALL** 

    ![](https://images.gitee.com/uploads/images/2020/1128/121157_37d3b82d_7401379.png "Screenshot.png")  
    Run the following commands to remove the write permission on the **/etc/sudoers** file and switch to the common user:  
     **chmod u-w /etc/sudoers**  
     **exit**

3. Connect the Atlas 200 DK to the Internet.

    **sudo vi /etc/netplan/01-netcfg.yaml**   
    Set the following parameters.  
    **Note: The configuration of both Netplan and Python is indentation-sensitive.** 

    ```
    network:
      version: 2
    #  renderer: NetworkManager
      renderer: networkd
      ethernets:
        eth0:
          dhcp4: yes 
       
        usb0:
          dhcp4: no 
          addresses: [192.168.1.2/24] 
          gateway4: 192.168.0.1
    ```
    Connect the Atlas 200 DK to the Internet with a network cable, and run the following command for the configuration to take effect:   
    **sudo netplan apply**      

4. Update the apt source for the Atlas 200 DK.

     **Select either of the following two sources. If the source fails to be updated, replace it with the Ubuntu 18.04 (ARM) source.** 

- Huawei Ubuntu 18.04 (ARM) source  

  Run the following command to change the source.  
  **sudo wget -O /etc/apt/sources.list https://repo.huaweicloud.com/repository/conf/Ubuntu-Ports-bionic.list --no-check-certificate**   

  Update the sources.  
  **sudo apt-get update** 

- Ubuntu 18.04 (ARM) source 

    Modify the source file.  
    **sudo vi /etc/apt/sources.list**   

    Replace the source file content with the following Ubuntu (ARM) sources:
    ```
    deb http://ports.ubuntu.com/ bionic main restricted universe multiverse
    deb-src http://ports.ubuntu.com/ bionic main restricted universe multiverse
    deb http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse
    deb-src http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse
    deb http://ports.ubuntu.com/ bionic-security main restricted universe multiverse
    deb-src http://ports.ubuntu.com/ bionic-security main restricted universe multiverse
    deb http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse
    deb-src http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse
    deb http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted
    deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted  
    ```

    Update the sources.  
    **sudo apt-get update** 

5. Add environment variables to the operating environment to run the project.
    1. Open the **.bashrc** file.  
        **vim ~/.bashrc** 

        Add the following environment variables to the file.  
        **export LD_LIBRARY_PATH=/home/HwHiAiUser/ascend_ddk/arm/lib:/home/HwHiAiUser/Ascend/acllib/lib64:\$LD_LIBRARY_PATH**  

        **export PYTHONPATH=/home/HwHiAiUser/Ascend/pyACL/python/site-packages/acl:\$PYTHONPATH** 

        Save the configuration and exit.  
        **wq!**   

     2. Make the configuration take effect.  
        **source ~/.bashrc**
