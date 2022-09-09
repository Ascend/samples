## CANN基础环境变量和Python环境变量配置说明（C++样例）

### 前置条件
已在环境上部署昇腾AI软件栈，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。 


### 安装须知
开发环境及运行环境说明如下：
- **运行环境：** 运行环境指可运行算子、推理或训练等程序的环境，运行环境必须带昇腾AI处理器的设备。       
- **开发环境：** 可用于代码开发、调试、编译等开发活动。该环境可以是带昇腾AI处理器的设备，也可以是其他满足CANN软件安装的环境。

开发环境和运行环境可以合设在同一台服务器上，也可以分设，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若开发环境和运行环境上的操作系统架构不同，则需要在开发环境中执行交叉编译。     

### 操作步骤

1.  以运行用户登录安装CANN软件的环境。

2.  配置CANN基础环境变量。

    1.  （可选）查看set_env.sh脚本中涉及哪些环境变量。

        根据环境上安装的软件包，到对应的安装目录下，查看set_env.sh脚本。此处以普通用户的默认安装路径为例:

        - 若环境上安装的是Ascend-cann-toolkit软件包，则在$HOME/Ascend/ascend-toolkit目录下查看set_env.sh脚本；

        - 若环境上安装的是Ascend-cann-nnrt软件包，则在$HOME/Ascend/nnrt目录下查看set_env.sh脚本。

    2.  （可选）执行以下命令查看查看环境变量的值是否正确。

        例如，set_env.sh脚本中有ASCEND_HOME_PATH环境变量，则执行“echo $ASCEND_HOME_PATH”命令，查看当前环境中该环境变量的值与set_env.sh脚本中的环境变量值是否一致，若一致，则跳至[配置Python环境变量](#li001)；若不一致，则跳至[设置CANN基础环境变量](#li002)后，再继续检查Python环境变量。   

    3.  设置CANN基础环境变量。<a name="li002"></a>

        当前昇腾CANN环境中已提供进程级环境变量设置脚本，供用户在进程中引用，以自动完成环境变量设置。用户进程结束后自动失效。示例如下（普通用户的默认安装路径为例）：

        - 安装Ascend-cann-toolkit包时配置
            ```
            . $HOME/Ascend/ascend-toolkit/set_env.sh
            ```

        - 安装Ascend-cann-nnrt包时配置
            ```
            . $HOME/Ascend/nnrt/set_env.sh
            ``` 

        用户也可以通过修改 **~/.bashrc** 文件方式设置永久环境变量，操作如下：

           （1）以运行用户在任意目录下执行 **vi ~/.bashrc** 命令，打开.bashrc文件，在文件最后一行后面添加上述内容。

           （2）执行 **:wq!** 命令保存文件并退出。

           （3）执行 **source ~/.bashrc** 命令使其立即生效。


3.  配置Python环境变量（以Python3.7.5版本的默认安装路径为例）。<a name="li001"></a>

    1.  （可选）执行 **echo $LD_LIBRARY_PATH** 命令，若返回的环境变量值中包含对应Python版本的lib库配置（例如，/usr/local/python3.7.5/lib），则Python版本的lib库配置正确；否则，跳至[设置Python环境变量](#li0003)。

    2.  （可选）执行 **echo $PATH** 命令，若返回的环境变量值中包含对应Python版本的bin配置（例如，/usr/local/python3.7.5/bin），则Python版本的bin配置正确；否则，跳至[设置Python环境变量](#li0003)。  

    3.  设置Python环境变量。<a name="li0003"></a> 

         ```
         #用于设置python3.7.5库文件路径
         export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
         #如果用户环境存在多个python3版本，则指定使用python3.7.5版本
         export PATH=/usr/local/python3.7.5/bin:$PATH
         ```
