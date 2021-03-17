中文|[English](README_200DK_EN.md)

# 安装Presenter Agent<a name="ZH-CN_TOPIC_0228768065"></a>
$\color{red}{以下命令在开发环境上用安装开发套件包的用户执行}$
1.  安装autoconf、automake、libtool依赖   
    **sudo apt-get install autoconf automake libtool python3-pip**
2.  安装python库  
    
    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**    
    **python3.6 -m pip install tornado==5.1.0 protobuf Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.7.5 -m pip install tornado==5.1.0 protobuf Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**
  
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
    >  **若Python包安装失败，可以试用其他源 https://bbs.huaweicloud.com/forum/thread-97632-1-1.html 或不加-i 参数使用默认pip源** 
3.  安装protobuf
    - 开发环境未安装在Atlas200DK上，需要交叉编译protobuf   
        **cd \$HOME**     
        **git clone -b 3.8.x https://gitee.com/mirrors/protobufsource.git protobuf**  
        **cp -r protobuf protobuf_arm**  
        **cd protobuf**  
        **./autogen.sh**  
        **bash configure**  
        **make -j8**  
        **sudo make install**  
        **cd \$HOME/protobuf_arm**  
        **./autogen.sh**  
        **./configure --build=x86_64-linux-gnu --host=aarch64-linux-gnu --with-protoc=protoc --prefix=$HOME/ascend_ddk/arm**  
        **make -j8**  
        **make install**  

    - 开发环境安装在Atlas200DK上，只需编译一次protobuf   
        **cd \$HOME**     
        **git clone -b 3.8.x https://gitee.com/mirrors/protobufsource.git protobuf**   
        **cd protobuf**  
        **./autogen.sh**  
        **./configure --prefix=$HOME/ascend_ddk/arm**  
        **make -j8**  
        **sudo make install**  
  
4.  编译并安装Presenter Agent
  
    设置环境变量，在命令行内执行   
    export DDK_PATH=\$HOME/Ascend/ascend-toolkit/latest/**_ARCH_**   
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
    >- 请将\$HOME/Ascend/ascend-toolkit/latest替换为ACLlib安装包的实际安装路径。   
    >- 若版本为3.0.0，请将 **ARCH** 替换为arm64-linux_gcc7.3.0；若版本为3.1.0，请将 **ARCH** 替换为arm64-linux。  

    下载Presenter Agent源码   
     **cd \$HOME**   
     **git clone https://gitee.com/ascend/samples.git**  
     **cd \$HOME/samples/cplusplus/common/presenteragent/**  

    安装Presenter Agent   
    **make -j8**   
    **make install**  

5.  将编译好的so传到运行环境    
    **scp \$HOME/ascend_ddk/arm/lib/libpr\* HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm/lib/**     


 
