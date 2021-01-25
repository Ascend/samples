English|[中文](README_200DK_CN.md)

# Installing Presenter Agent
$\color{red}{Run the following commands in the development environment as the user who installs Toolkit.}$
1. Install the Autoconf, Automake, and Libtool dependencies.   
    **sudo apt-get install autoconf automake libtool python3-pip**
2. Install Python libraries.  

    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**    
    **python3.6 -m pip install tornado==5.1.0 protobuf Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.7.5 -m pip install tornado==5.1.0 protobuf Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**

    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >**If Python fails to be installed, click [here](https://bbs.huaweicloud.com/forum/thread-97632-1-1.html) to try a new source. Alternatively, use the default pip source by removing the the -i option from the command.** 
3. Install Protobuf. (Run the following commands in sequence. Because cross compilation is required, you need to compile Protobuf twice.)  

    **cd $HOME**     
    **git clone -b 3.8.x https://gitee.com/mirrors/protobufsource.git protobuf**  
    **cp -r protobuf protobuf_arm**  
    **cd protobuf**  
    **./autogen.sh**  
    **bash configure**  
    **make -j8**  
    **sudo make install**  
    **cd $HOME/protobuf_arm**  
    **./autogen.sh**  
    **./configure --build=x86_64-linux-gnu --host=aarch64-linux-gnu --with-protoc=protoc --prefix=$HOME/ascend_ddk/arm**  
    **make -j8**  
    **make install**    

4. Build and install Presenter Agent.

    Set the environment variable and run the following command on the command line.   
    **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/_ARCH_**   
    >![](public_sys-resources/icon-note.gif) **NOTE**  
    >
    >Replace ***$HOME/Ascend/ascend-toolkit/latest*** with the actual ACLlib installation path.   
    >For CANN 20.0, replace ***ARCH*** with **arm64-linux_gcc7.3.0**. For CANN 20.1, replace ***ARCH*** with **arm64-linux**.   
    Download the Presenter Agent source code.   
     **cd $HOME**   
     **git clone https://gitee.com/ascend/samples.git**  
     **cd $HOME/samples/cplusplus/common/presenteragent/**  

    Install Presenter Agent.   
    **make -j8**   
    **make install**  

5. Upload the built .so file to the operating environment.    
    **scp $HOME/ascend_ddk/arm/lib/libpresenteragent.so HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm/lib/**     


 