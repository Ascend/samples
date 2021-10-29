**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.1.0及以上版本，支持产品为Atlas200DK和Atlas300。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## RTSP视频流目标检测样例

功能：使用yolov3目标检测模型对输入视频进行目标检测。

样例输入：网路摄像头rtsp视频流或者H264文件。

样例输出：终端界面直接打印检测结果。


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[atlasutils部署说明](../../../common/atlas_utils)准备好运行环境，并将部署路径添加到PYTHONPATH环境变量中。

- 使用本地视频文件测试本样例时，要求视频文件为h264裸流文件，并且为annex-b格式。mp4转h264可以使用命令：

  ffmpeg -i aaa.mp4 -codec copy -bsf: h264_mp4toannexb -f h264 aaa.h264

- 在使用pyav读取视频切帧时对mp4文件有另外约束，可以参考[ffmpeg指令使MP4文件满足pyav和dvpp的约束](https://bbs.huaweicloud.com/forum/thread-131548-1-1.html)中的指令使mp4文件满足其约束


### 工程准备

1. 非root用户进入运行环境，在命令行中执行以下命令下载源码仓。

   **cd $HOME**

   **git clone https://github.com/Ascend/samples.git**

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/coco_detection_rtsp。

    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  yolov3 | 是基于Caffe的yolov3目标检测模型。|  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
   
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/coco_detection_rtsp**  

        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/aipp_nv12.cfg**

        **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_yuv --soc_version=Ascend310 --insert_op_conf=aipp_nv12.cfg**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./yolov3_yuv.om $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp/model/**      

### 样例运行

1. 修改配置文件。
   将文件中[videostream]段下添加要检测的视频文件或者rtsp地址. 本样例最多支持6路视频流

2. 部署样例源码。执行以下命令,将开发环境的 **coco_detection_rtsp** 目录或者整个samples目录上传到运行环境中，例如 **/home/HwHiAiUser**。   

```
scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser  
scp -r $HOME/samples/python/common/atlas_utils HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser/coco_detection_rtsp
```
   其中xxx.xxx.xxx.xxx为运行环境ip，例如atlas200dk usb虚拟网口连接时默认地址为192.168.1.2。 下同，不在赘述

3. 登录运行环境，执行样例

```
ssh HwHiAiUser@xxx.xxx.xxx.xxx
cd $HOME/coco_detection_rtsp/src
python3.6 main.py
```



### 查看结果

终端打印每一路的检测结果。
