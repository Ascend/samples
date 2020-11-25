## cartoonGAN图片版本样例

**本样例只针对教学使用，禁止一切商业活动！**

**运行本样例前，请确认已安装好20.0.0版本的环境**。

**样例适配Atlas200DK及Atlas300，运行过程中，有任何问题，请直接在本仓中提issue，我们会及时解决，谢谢！**

### 样例介绍

功能：使用cartoonGAN模型对输入图片进行卡通化处理。

样例输入：待推理的jpg图片。

样例输出：推理后的jpg图片。

### 工程准备

1. 非root用户命令行中执行以下命令下载源码仓。

   **cd $HOME**

   **git clone https://gitee.com/ascend/samples.git**

2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。    
 
     -  下载原始网络模型至ubuntu服务器任意目录，如:$HOME/cartoon。

        **mkdir -p $HOME/cartoon**

        **wget -P $HOME/cartoon https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/cartoon/cartoonization.pb** 

 3. 将原始网络模型转换为适配昇腾AI处理器的模型。

    1.  在Mind Studio操作界面的顶部菜单栏中选择**Tools \> Model Converter**，进入模型转换界面。
    2.  在弹出的**Model Conversion**操作界面中，进行模型转换配置。
    3.  参照以下图片进行参数配置。    
        -   Model File选择步骤2中下载的模型文件。 
        -   Arguments 填写参数  **--precision_mode=allow_fp32_to_fp16**  。
        -   Input Type 选择 **FP32** 。  
        -   Mean、Min、Variance分别填写 **127**、**0.5**、**0.0039216** 。

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1125/165533_6f67c648_7401379.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1125/165957_81204f0b_7401379.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1125/165253_20b5c460_7401379.png "屏幕截图.png")
4. 将转换好的模型文件（.om文件）上传到步骤1中源码所在路径下的“cartoonGAN_picture/model”目录下。

    
     **cp $HOME/modelzoo/cartoonization/device/cartoonization.om ~/samples/contrib/cartoonGAN_picture/model** 
    

5. 将应用代码拷贝到运行环境。  

    **scp -r ~/samples/contrib/cartoonGAN_picture  HwHiAiUser@X.X.X.X:/home/HwHiAiUser/HIAI_PROJECTS**  
 
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > -  **X.X.X.X请替换为运行环境的ip，如USB方式连接的200DK为192.168.1.2，ai1s云端推理环境为公网ip地址。** 

6. 将acl.so拷贝到运行环境。

   **scp ${HOME}/Ascend/ascend-toolkit/X.X.X/X_64-linux_gcc7.3.0/pyACL/python/site-packages/acl/acl.so HwHiAiUser@X.X.X.X:/home/HwHiAiUser/Ascend/**  
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**     
   >- **请将X.X.X替换为Ascend-Toolkit开发套件包的实际版本号。**   
      **例如：Toolkit包的包名为Ascend-Toolkit-20.0.RC1-x86_64-linux_gcc7.3.0.run，则此Toolkit的版本号为20.0.RC1。**
   > -  **如为200DK，请将X_64-linux_gcc7.3.0替换为arm64-linux_gcc7.3.0,如为ai1s云端推理环境，请将X_64-linux_gcc7.3.0替换为X86_64-linux_gcc7.3.0** 
   > -  **X.X.X.X请替换为运行环境的ip，如USB方式连接的200DK为192.168.1.2，ai1s云端推理环境为公网ip地址。** 

7. 登录开发环境，添加环境变量。  

   **ssh HwHiAiUser@X.X.X.X**  
   **vim \${HOME}/.bashrc**   
   在最后添加两行    
   **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64**   
   **export PYTHONPATH=/home/HwHiAiUser/Ascend/:\\${PYTHONPATH}**  
   ![](figures/pythonpath.png)   
   执行如下命令，使环境变量生效   
   **source \${HOME}/.bashrc**

8. 安装环境依赖。

   - 安装python opencv  
       Atlas200DK 请参考 https://gitee.com/ascend/samples/tree/master/common/install_opencv/for_atlas200dk 进行安装。 
       Atlas300 请参考https://gitee.com/ascend/samples/tree/master/common/install_opencv/for_atlas300 进行安装。
### 样例运行

   以HwHiAiUser用户登录运行环境，进入工程目录下，执行如下命令运行程序。  

   **cd ~/HIAI_PROJECTS/cartoonGAN_picture/**  

   **python3 cartoonization.py ./data/** 
​       

### 查看结果

运行完成后，会在outputs目录下生成带推理结果的jpg图片。
