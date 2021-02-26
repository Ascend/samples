English|[中文](README_300_CN.md)

# Basic Environment Configuration  
This readme file describes how to configure the basic environment, including the sudo permission, apt source, and environment variables. If they have been configured, skip this workflow.  

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
    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >**After the dependency installation, you can cancel the sudo permission by yourself.**

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


3. Add the following environment variables to the development environment for ATC model conversion. 

    1. Open the **.bashrc** file.  
        **vim ~/.bashrc** 

        Add the following environment variables to the file.  
        - For CANN 20.0 

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**
    
            **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  
    
            **export ASCEND_OPP_PATH=\\${install_path}/opp**  
   
            **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

            **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**   
            
    
        - For CANN 20.1 

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**
    
            **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  
    
            **export ASCEND_OPP_PATH=\\${install_path}/opp**  
   
            **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  
          
            **export PYTHONPATH=\\${install_path}/atc/python/site-packages:\\${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:\\${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH**  
        >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
        >**- If the development environment and operating environment are set up on the same server, do not configure LD_LIBRARY_PATH, avoiding the conflict with LD_LIBRARY_PATH in the operating environment when running the sample.** 

        - For CANN 20.2  

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest** 

            **export PATH=\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

            **export ASCEND_OPP_PATH=\${install_path}/opp**  

            **export ASCEND_AICPU_PATH=\${install_path}** 

        >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
        >**- Replace install_path with the actual installation path.**  

    2. Make the configuration take effect.   
        **source ~/.bashrc**  

Run the following commands in the operating environment.
1. Log in to the operating environment.  

2. Add environment variables to the operating environment to run the project.
    1. Open the **.bashrc** file.  
        **vim ~/.bashrc** 

        Add the following environment variables to the file.  

        **export PYTHONPATH=\\$HOME/Ascend/nnrt/latest/pyACL/python/site-packages/acl:\\$PYTHONPATH**  

        The environment variable ***LD_LIBRARY_PATH*** is incompatible with CANN 20.0 and 20.1. Run the corresponding command to add the ***LD_LIBRARY_PATH*** environment variable based on the CANN version.
        - For CANN 20.0

            **export LD_LIBRARY_PATH=\\$HOME/ascend_ddk/x86/lib:\\$HOME/Ascend/nnrt/latest/acllib_linux.x86_64/lib64:\\$LD_LIBRARY_PATH**

        - For CANN 20.1

            **export LD_LIBRARY_PATH=\\$HOME/ascend_ddk/x86/lib:\\$HOME/Ascend/nnrt/latest/acllib/lib64:\\$LD_LIBRARY_PATH**

        Save the configuration and exit.  
        **wq!**   
        >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
        >  **In this example, the CANN software package is installed by a non-root user.**      

     2. Make the configuration take effect.  
        **source ~/.bashrc**