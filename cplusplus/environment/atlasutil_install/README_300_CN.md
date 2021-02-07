中文|[English](README_300_EN.md)
# atlasutil库使用说明<a name="ZH-CN_TOPIC_0228768065"></a>
1. atlasutil库对当前开源社区样例中

    - Atlas200DK板载摄像头

    - acl dvpp图像和视频处理

    - acl模型推理等进行封装

    等重复代码进行封装，提供一组公共接口。


2. 本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；仅支持Atlas200DK和Atlas300样例。

# 部署方法

$\color{red}{以下命令在开发环境上用安装开发套件包的用户执行}$
1.  下载源码  
 **cd $HOME**   
 **git clone https://gitee.com/ascend/samples.git**   

2.  设置环境变量  
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/**_ARCH_**   
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**    
         请将$HOME/Ascend/ascend-toolkit/latest替换为ACLlib安装包的实际安装路径。   
         若版本为20.0，请将 **ARCH** 替换为x86_64-linux_gcc7.3.0；若版本为20.1，请将 **ARCH** 替换为x86_64-linux。 

3.  编译并安装atlasutil   
    **cd $HOME/samples/cplusplus/common/atlasutil/**      
    **make mode=ASIC**  
    **make mode=ASIC install**  
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
    >  **生成的libatalsutil.so在\\$HOME/ascend_ddk/x86/lib/下；头文件在\\$HOME/ascend_ddk/x86/include/atlasutil下。**   

4.  将编译好的so传到运行环境 (如开发环境和运行环境安装在同一服务器，请忽略此步)   
    **scp \$HOME/ascend_ddk/x86/lib/libatlasutil.so HwHiAiUser@_IP_:/home/HwHiAiUser/ascend_ddk/x86/lib/**   

    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
     请将IP替换为ai1s云端推理环境的公网ip地址，HwHiAiUser替换为实际的运行用户。

    
 

 

