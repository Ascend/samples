中文|[英文](README_200dk_EN.md)  
# 基础环境配置  
本文的目的是进行基础环境配置，包含sudo权限配置、apt源配置、开发者板联网、环境变量配置。如已配置，均可跳过。  

$\color{red}{以下操作在开发环境上操作，以普通用户为HwHiAiUser为例，请根据实际情况进行修改。}$
  

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


1. 在开发环境安装编译工具 

     **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu**   

2. 在开发环境中添加以下环境变量，用于atc模型转换 
    
    1.  打开.bashrc文件  
        **vim ~/.bashrc** 
      
        在文件中添加以下环境变量  
        **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**
    
        **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  
    
        **export ASCEND_OPP_PATH=\\${install_path}/opp**  
   
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64** 
    
        其中，PYTHONPATH环境变量在20.0和20.1版本不兼容。请按照CANN版本选择对应方式执行命令添加PYTHONPATH环境变量。
    
        - 20.0版本
          
            **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**  
    
        - 20.1版本
          
            **export PYTHONPATH=\\${install_path}/atc/python/site-packages:\\${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:\\${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH**  
 
        保存退出  
        **wq!**
        >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
        >**- install_path 请根据实际情况修改。**  
        >**- 若开发环境与运行环境部署在一台服务器上时，请勿配置LD_LIBRARY_PATH，在运行样例时，会跟运行环境的LD_LIBRARY_PATH有冲突。**

    2.  执行如下命令使环境变量生效   
        **source ~/.bashrc**  
  
$\color{red}{以下操作在运行环境(Atlas200DK)上操作}$  
1.  登录运行环境  
    ssh HwHiAiUser@X.X.X.X  

2.  给HwHiAiUser用户配置sudo权限
    

    切换为root用户 （root用户默认密码：Mind@123）   
    **su root**

    给sudoer文件配置写权限，并打开该文件  
    **chmod u+w /etc/sudoers**   
    **vi /etc/sudoers** 

    在该文件 `# User privilege specification` 下面增加如下内容：  
     **HwHiAiUser    ALL=(ALL:ALL) ALL** 

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1128/121157_37d3b82d_7401379.png "屏幕截图.png")  
    完成后，执行以下命令取消`/etc/sudoers`文件的写权限，并切换回普通用户  
     **chmod u-w /etc/sudoers**  
     **exit**

3.  开发者板设置联网
     
    **sudo vi /etc/netplan/01-netcfg.yaml**   
    填写以下配置  
    **注：需要注意这里的缩进格式，netplan配置时和python类似，对缩进有强限制** 

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
   

    将开发板网口接上可正常联网的网线，执行以下命令使配置生效   
    **sudo netplan apply**      
  
4.  开发者板apt换源配置

     **以下给出两种源，选择其中一种使用，如更新源失败，请自行更换可用Ubuntu 18.04 arm源** 
    - ubuntu18.04-arm华为源  
 
        执行以下换源操作  
        **sudo wget -O /etc/apt/sources.list https://repo.huaweicloud.com/repository/conf/Ubuntu-Ports-bionic.list**   

        更新源  
        **sudo apt-get update** 

    - ubuntu18.04-arm官方源 

        修改源文件  
        **sudo vi /etc/apt/sources.list**   
         
        将源文件内容替换为以下ubuntu-arm官方源。
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


        更新源。  
        **sudo apt-get update** 

5.  在运行环境添加环境变量，用于运行工程。
    1.  打开.bashrc文件  
        **vim ~/.bashrc** 
      
        在文件中添加以下环境变量  
        **export LD_LIBRARY_PATH=/home/HwHiAiUser/ascend_ddk/arm/lib:/home/HwHiAiUser/Ascend/acllib/lib64:$LD_LIBRARY_PATH**  
        
        **export PYTHONPATH=/home/HwHiAiUser/Ascend/pyACL/python/site-packages/acl:$PYTHONPATH** 
        
        保存退出  
        **wq!**   
        

     2.  执行如下命令使环境变量生效。  
        **source ~/.bashrc**
 