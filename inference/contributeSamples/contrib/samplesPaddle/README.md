## 目录

  - [样例介绍](#样例介绍)
  - [docker安装运行环境](#docker安装运行环境)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

  演示如何在昇腾310上使用docker镜像配置环境，并通过编译好的paddlelite-demo包，在npu上使用paddle-lite做模型推理应用。  
  图片分类：  
  &nbsp;&nbsp;    - 样例输入：图片  
  &nbsp;&nbsp;    - 样例输出：图片，分类结果。  

## docker安装运行环境

  - 系统架构amd64下安装docker
    ```
    sudo apt-get install docker
    ```
  - 下载samples仓，根据本目录下的docker文件安装镜像，命名 ascend_x86，命令如下：
    ```
    git clone https://github.com/Ascend/samples.git
    cd samples/inference/contributeSamples/contrib/samplesPaddle
    docker build --network=host -f Ascend_ubuntu18.04_x86_5.1.rc1.alpha001.Dockerfile -t paddlelite/ascend_x86:cann_5.1.1.alpha001 .
    ```
  - 使用安装好的镜像文件创建docker容器，创建时需要挂载/usr/local目录下的驱动，命令如下：
    ```
    docker run -itd --privileged --name=ascend-x86 --net=host -v $PWD:/Work -w /Work --device=/dev/davinci0 --device=/dev/davinci_manager --device=/dev/hisi_hdc --device=/dev/devmm_svm -v /usr/local/bin/npu-smi:/usr/local/bin/npu-smi  -v /usr/local/Ascend/driver/:/usr/local/Ascend/driver/ paddlelite/ascend_x86:cann_5.1.1.alpha001 /bin/bash
    ```
  - 进入到docker容器内，输入命令查看驱动是否挂载成功
    ```
    docker exec -it ascend-x86 /bin/bash
    npu-smi info
    ```

## 样例运行
  [Paddle](https://www.paddlepaddle.org.cn/lite/v2.12/demo_guides/huawei_ascend_npu.html)通过提供编译好的demo包，可快捷实现在npu上的模型推理应用，包括物体分类，目标检测，人脸识别，姿态检测等等，此处我们选用提供的 resnet50 演示如何在npu上进行图片分类

#### 获取模型和数据
  - 在docker容器内下载Paddle-lite-demo包
    ```
    wget https://paddlelite-demo.bj.bcebos.com/devices/generic/PaddleLite-generic-demo_v2_12_0.tar.gz
    tar -zxvf PaddleLite-generic-demo_v2_12_0.tar.gz
    ```

#### 进行resnet50推理测试
  进入image_classification_demo/shell目录，执行run.sh脚本，打印分类结果top_5，并输出图片。  
  - 运行
    ```
    cd PaddleLite-generic-demo/image_classification_demo/shell
    ./run.sh resnet50_fp32_224 imagenet_224.txt test linux amd64 huawei_ascend_npu
    ```    

  - 输出结果  
    ```
    Top1 tabby, tabby cat - 0.705078
    Top2 tiger cat - 0.134521
    Top3 Egyptian cat - 0.121582
    Top4 lynx, catamount - 0.028870
    Top5 ping-pong ball - 0.001052
    [0] Preprocess time: 2.247000 ms Prediction time: 3.250000 ms Postprocess time: 3.545000 ms
    Preprocess time: avg 2.247000 ms, max 2.247000 ms, min 2.247000 ms
    Prediction time: avg 3.250000 ms, max 3.250000 ms, min 3.250000 ms
    Postprocess time: avg 3.545000 ms, max 3.545000 ms, min 3.545000 ms
    ```    
    ![输出图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/contributeSamples/paddle/tabby_cat_output.jpg "tabby_cat_output.jpg")              

## 其他资源

以下资源提供了对OpenCV项目和昇腾NPU的更深入理解：

**指导文档**
- [paddle：ascend](https://www.paddlepaddle.org.cn/lite/v2.12/demo_guides/huawei_ascend_npu.html)

**Paddle-Lite仓**
- [GitHub: PaddlePaddle](https://github.com/PaddlePaddle/Paddle-Lite/)

**Documentation**

- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/04/24 | 更新ReadMe.md|
  
## 已知issue
   无
