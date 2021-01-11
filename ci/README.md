# 1.脚本介绍

​	该脚本是一个用于快速下载，编译，运行，检测Atlas200DK、Atlas300样例仓的工具。整个文件包含一个dev_file_build.config配置文件,和一些sh格式的脚本文件。

# 2.配置文件

​	配置文件中目前有如下配置项

​	**project**：需要进行自动化部署的工程的名称。填写单个或多个工程的名称，用空格隔开。如果需要全部编译，那么只填all

​	**ACLlib_install_path**: ACLlib标准形态安装包的实际安装路径。例如本地安装路径为$HOME/Ascend/ascend-toolkit/20.0.RC1,那么这里的值就是Ascend/ascend-toolkit/20.0.RC1

​	**IP_address**:运行环境的IP地址,如192.168.1.2

​	**TargetTarchitecture**：指定编译版本aarch64(Atlas200DK)或者x86_64(ai1s云端推理环境)

​	**build_all_project**：可以进行自动化部署的所有工程的名称。一般情况下不需要做修改。名称之间也使用空格分开。

​	**run_project_yes_or_not**：是否需要进行运行。填写yes会在编译结束后开始运行。填写其他则不会运行。

配置文件示例如下

```shell
project=googlenet_imagenet_picture googlenet_imagenet_multi_batch

ACLlib_install_path=Ascend/ascend-toolkit/20.0.RC1

IP_address=192.168.1.2

TargetTarchitecture=aarch64

build_all_project=venc vdec gpio  i2c crop googlenet_imagenet_picture googlenet_imagenet_multi_batch

run_project_yes_or_not=yes
```



# 3.编译脚本

编译脚本是dev_file_build.sh，这个文件主要功能是进行自动下载代码，下载模型，然后进行编译。另外通过配置文件，可以选择是否自动运行样例。

在终端进入dev_file_build.sh所在的路径，授予dev_file_build.sh运行权限，然后开始运行。

```shell
chmod +x dev_file_build.sh
chmod +x copy_files_to_device.sh
./dev_file_build.sh
```



# 4.运行脚本

## 1.copy_files_to_device.sh

这个脚本的作用是将编译得到的可执行文件以及依赖文件推送到运行环境中。如果前边在配置文件中设置了自动运行的话，那么这里就被调用并运行。如果没有设置自动运行，那么也可以在这里使用手动执行。首先在终端中进入到脚本所在路径，给脚本授予运行权限，然后开始运行。如下：

```
chmod +x copy_files_to_device.sh
./copy_files_to_device.sh
```



## 2.device_run_test.sh

这里需要强调的是，这里是**需要登录到运行环境**来运行。所以在使用之前需要保证能够正常登录到运行环境。

以Atlas200DK为例，假设IP为192.168.1.2，那么需要在终端切换到运行环境：

```shell
ssh HwHiAiUser@192.168.1.2
```

copy_files_to_device.sh已经将所需文件推送到运行环境中，分别位于/home/HwHiAiUser/run_in_device和/home/HwHiAiUser/project_run_test_temp/中。登录到开发板后，进入到run_in_device，给sh脚本添加运行权限，并开始执行。

```shell
chmod +x 2_device_run_test.sh
./device_run_test.sh
```

之后，脚本会开始运行。将推理结果输出到/home/HwHiAiUser/project_run_test_temp/run_output_temp，并以样例的工程名命名。
