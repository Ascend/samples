# sample-colorization-python

#### 介绍
1.3X版本黑白图像上色python样例

#### 软件架构
atlasutil： Atlask开发板python工具包,当前支持摄像头、YUV420SP编码为jpeg、hiai推理和发送展示视频图像的presentagent

presenterserver： presenter服务端，在ubuntu服务器上运行

model：  黑白图像上色离线模型（om模型）路径

colorization.py： 黑白图像上色实现

resource： 图片输入路径


#### 环境准备
1. 在开发板上安装hiai库，参见：https://bbs.huaweicloud.com/forum/thread-35192-1-1.html
2. 在开发板上安装OpenCV，参见：https://bbs.huaweicloud.com/forum/thread-47355-1-1.html


#### 代码部署
部署代码到开发板上，您可以采取一下两种方式的任一种：
- 下载代码到Ubuntu宿主机，拷贝到开发板
    scp -r sample-colorization-python HwHiAiUser@192.168.1.2
- 直接下载到开发板
    git clone https://gitee.com/Atlas200DK/sample-colorization-python.git
    ```
    注：这种方式需要配置开发板联网，并安装git。开发板联网参见：https://bbs.huaweicloud.com/forum/thread-26546-1-1.html。
    安装git命令：apt-get install git
    ```

#### 运行
1.  模型转换，参见：https://gitee.com/Atlas200DK/sample-colorization 中模型转换，并把转换好的模型放在model目录下

2. 将待上色的图像放在resource目录下
3. 运行colorization.py，可以在result目录中查看上色结果