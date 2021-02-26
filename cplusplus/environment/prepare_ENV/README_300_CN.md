中文|[English](README_300_EN.md)  

# 基础环境配置  
本文的目的是进行基础环境配置，包含sudo权限配置、apt源配置、环境变量配置。如已配置，均可跳过。  

$\color{red}{以下命令在开发环境上执行，以普通用户为HwHiAiUser为例，请根据实际情况进行修改。}$
  

1.  给HwHiAiUser用户配置sudo权限
    

    切换为root用户  
     **su root** 

    给sudoer文件配置写权限，并打开该文件  
     **chmod u+w /etc/sudoers**   
     **vi /etc/sudoers** 

    在该文件 `# User privilege specification` 下面增加如下内容：  
     **HwHiAiUser ALL=(ALL:ALL) ALL** 

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1128/144046_7c02d0d0_7401379.png "屏幕截图.png")

    完成后，执行以下命令取消`/etc/sudoers`文件的写权限  
     **chmod u-w /etc/sudoers**    
    切换回普通用户  
     **exit**  
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**    
    >**用户在完成环境依赖安装后，可自行取消sudo权限。**

2.  apt源配置  

    配置ubuntu18.04-x86的apt清华源
   
    **sudo vi /etc/apt/sources.list** 

    将源文件内容替换为以下ubuntu18.04-x86的apt清华源

    ```
    # 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
    ```
    执行以下命令更新源   
    **sudo apt-get update** 
    <br/> </br>
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
    >  **如果sudo apt-get update失败，可以试用其他的国内源 https://www.cnblogs.com/dream4567/p/9690850.html** 


3. 在开发环境中添加以下环境变量，用于atc模型转换 
    
    1.  打开.bashrc文件  
        **vim ~/.bashrc** 
      
        在文件中添加以下环境变量  
        - 20.0版本  

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**
    
            **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  
    
            **export ASCEND_OPP_PATH=\\${install_path}/opp**  
   
            **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

            **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**   
            
    
        - 20.1版本  

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**
    
            **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  
    
            **export ASCEND_OPP_PATH=\\${install_path}/opp**  
   
            **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  
          
            **export PYTHONPATH=\\${install_path}/atc/python/site-packages:\\${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:\\${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH**  
 

        >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**    
        >**若开发环境与运行环境部署在一台服务器上时，请勿配置LD_LIBRARY_PATH，在运行样例时，会跟运行环境的LD_LIBRARY_PATH有冲突。**

        - 20.2版本 

            **export install_path=\\$HOME/Ascend/ascend-toolkit/latest** 

            **export PATH=\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

            **export ASCEND_OPP_PATH=\${install_path}/opp**  

            **export ASCEND_AICPU_PATH=\${install_path}**  
        >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**    
        >**install_path 请根据实际情况修改。**

    2.  执行如下命令使环境变量生效   
        **source ~/.bashrc**  
  
$\color{red}{以下命令在运行环境上执行}$  
1.  登录运行环境  

2.  在运行环境添加环境变量，用于运行工程。
    1.  打开.bashrc文件  
        **vim ~/.bashrc** 
      
        在文件中添加以下环境变量  

        **export PYTHONPATH=\\$HOME/Ascend/nnrt/latest/pyACL/python/site-packages/acl:$PYTHONPATH**  
 
        其中，LD_LIBRARY_PATH环境变量在20.0和20.1版本不兼容。请按照CANN版本选择对应方式执行命令添加LD_LIBRARY_PATH环境变量。
        - 20.0版本
          
            **export LD_LIBRARY_PATH=\\$HOME/ascend_ddk/x86/lib:\\$HOME/Ascend/nnrt/latest/acllib_linux.x86_64/lib64:$LD_LIBRARY_PATH**
    
        - 20.1版本
          
            **export LD_LIBRARY_PATH=\\$HOME/ascend_ddk/x86/lib:\\$HOME/Ascend/nnrt/latest/acllib/lib64:$LD_LIBRARY_PATH**
        
        保存退出  
        **wq!**   
        >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
        >  **此处的环境变量配置是以CANN软件包使用非root用户安装为例。**      

     2.  执行如下命令使环境变量生效。  
        **source ~/.bashrc**