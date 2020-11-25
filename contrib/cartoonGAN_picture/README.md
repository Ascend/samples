## cartoonGAN图片版本样例

**本样例只针对教学使用，禁止一切商业活动！**

**运行本样例前，请确认已安装好20.0.0及以上版本的环境**。

**样例适配Atlas200DK及Atlas300，运行过程中，有任何问题，请直接在本仓中提issue，我们会及时解决，谢谢！**

### 样例介绍

功能：使用cartoonGAN模型对输入图片进行卡通化处理。

样例输入：待推理的jpg图片。

样例输出：推理后的jpg图片。

### 工程准备

1. 非root用户命令行中执行以下命令下载源码仓。

   **cd $HOME**

   **git clone https://gitee.com/ascend/samples.git**

2. 准备om模型。

   在**ModelZoo**下载对应模型，获取模型文件，并放置到对应工程的model目录下。

3. 将应用代码拷贝到运行环境。  

    **scp -r ~/samples/python/contrib/cartoonGAN_picture  HwHiAiUser@X.X.X.X:/home/HwHiAiUser/HIAI_PROJECTS**  
 
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - X.X.X.X请替换为运行环境的ip，如USB方式连接的200DK为192.168.1.2，ai1s云端推理环境为公网ip地址。

### 样例运行

   以HwHiAiUser用户登录运行环境，进入工程目录下，执行如下命令运行程序。  

   **cd ~/HIAI_PROJECTS/cartoonGAN_picture/**  

   **python3 cartoonization.py ./data/** 
​       

### 查看结果

运行完成后，会在outputs目录下生成带推理结果的jpg图片。
