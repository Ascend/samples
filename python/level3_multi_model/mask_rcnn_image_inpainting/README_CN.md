**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 去除图像的指定前景目标样例

功能：使用MaskRcnn和ImageInpainting模型对输入图片指定前景目标去除推理。
样例输入：待去除目标的图片。
样例输出：生成的待去除的目标的mask图片、实例分割图片和去除目标后的图片。

### 适配要求

该样例运行，还需要执行以下命令安装程序所需要的python依赖
  ```
    python3.6 -m pip install mmcv --user
    python3.6 -m pip install easydict==1.9 --user
    python3.6 -m pip install matplotlib==3.3.4 --user
  ```
本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv, pillow，numpy，mmcv，easydict，matplotlib，python-acllite | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将zip包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip    

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  mask_rcnn+image_inpainting | 使用MaskRcnn和ImageInpainting模型对输入图片指定前景目标去除的模型。  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/googlenet/ATC_googlenet_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/googlenet/ATC_googlenet_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd ${HOME}/samples/python/level3_multi_model/mask_rcnn_image_inpainting/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/maskrcnn_mindspore.air
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/aipp_rgb.cfg     
    atc --input_format=NCHW --framework=1 --model="maskrcnn_mindspore.air" --input_shape="x:1, 3, 768, 1280; im_info: 1, 4" --output="maskrcnn_mindspore_rgb" --insert_op_conf="aipp_rgb.cfg" --precision_mode=allow_fp32_to_fp16 --soc_version=Ascend310 --op_select_implmode=high_precision
    wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json
    atc --singleop=matmul_27648.json --output=./ --soc_version=Ascend310  
    wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om
    ```

3. 获取样例需要的测试图片。
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd $HOME/samples/python/level3_multi_model/mask_rcnn_image_inpainting/data
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/photo1.jpg
    ```
### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **mask_rcnn_image_inpainting** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/python/level3_multi_model/mask_rcnn_image_inpainting HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/mask_rcnn_image_inpainting/src
    ```

2. <a name="step_2"></a>运行工程。
    ```
    python3.6 mask_rcnn.py 410 664
    ```    
### 查看结果

运行完成后，会在样例目录下创建mask和output文件夹，mask文件夹下生成实例分割图片，output文件夹下生成去除目标后的图片。

推理前：
![输入图片说明](https://images.gitee.com/uploads/images/2021/1110/141432_23bde59f_8083019.jpeg "photo1.jpg")

推理后：
![输入图片说明](https://images.gitee.com/uploads/images/2021/1110/141623_eee7745f_8083019.jpeg "photo1_out.jpg")
![输入图片说明](https://images.gitee.com/uploads/images/2021/1110/141612_746406cb_8083019.jpeg "photo1_mask.jpg")
![输入图片说明](https://images.gitee.com/uploads/images/2021/1110/141637_737d9472_8083019.jpeg "outpaint_photo1.jpg")