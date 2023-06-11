## 输入MP4视频文件，输出rtsp流显示的配置流程

1. 下载获取输入视频，放在data目录下
```
cd $HOME/samples/cplusplus/level3_application/1_cv/detect_and_classify/data
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
```

2.下载使用推流工具mediamtx
- 下载
    在[mediamtx下载页面](https://github.com/aler9/mediamtx/releases)选择自己合适的版本压缩包下载解压到任意目录（eg:/home/HwHiAiUser/）

- 使用
    重新打开一个窗口。下载解压后直接运行mediamtx可执行文件，它会自动绑定本地的IP，然后同时开启RTSP、RTMP、HLS三个服务。具体信息如下：
    ```
    2023/05/11 18:07:50 INF MediaMTX / rtsp-simple-server v0.22.0
    2023/05/11 18:07:50 INF [RTSP] listener opened on :8554 (TCP), :8000 (UDP/RTP), :8001 (UDP/RTCP)
    2023/05/11 18:07:50 INF [RTMP] listener opened on :1935
    2023/05/11 18:07:50 INF [HLS] listener opened on :8888
    2023/05/11 18:07:50 INF [WebRTC] listener opened on :8889 (HTTP)
    ```								
    服务开启后可以便可以使用ffmpeg进行推流到服务器上。
    ![输入图片说明1](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/mediamtx%E5%B7%A5%E5%85%B7%E4%BD%BF%E7%94%A8%E7%A4%BA%E4%BE%8B.png)

3. 修改配置文件中对应的URL、inputType_0、outputType_0、inputDataPath_0

```
# 切换到配置文件所在目录
cd $HOME/samples/cplusplus/level3_application/1_cv/detect_and_classify/scripts
# 执行如下命令获取推流ip地址
ifconfig
#打开配置文件params.conf根据ip地址进行修改
```
![输入图片说明2](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/detect_and_classify/out3.png)
![输入图片说明3](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/detect_and_classify/out.png)
```
[rtsp_options_param]
URL=rtsp://192.168.0.163:8554/stream0

[input_param_0]
inputType_0=video  #pic ; video ; rtsp
outputType_0=rtsp   #pic ; video ; presentagent ; stdout ; rtsp
inputDataPath_0=../data/car1.mp4
outputFrameWidth_0=1280
outputFrameHeight_0=720
```

4. 使用smplayer工具进行rtsp流展示
- 下载
    使用如下命令下载安装smplayer工具
    ```								
    sudo add-apt-repository ppa:rvm/smplayer 
    sudo apt-get update 
    sudo apt-get install smplayer smplayer-themes smplayer-skins
    ```							
- 使用
    重新打开一个窗口。输入如下命令启动smplayer工具：
    ```					
    smplayer
    ```
    ![输入图片说明4](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/smplayer%E5%90%AF%E5%8A%A8%E6%88%AA%E5%9B%BE%E7%A4%BA%E4%BE%8B.png)
    smplayer窗口右键open>URL,输入URL地址：rtsp://192.168.0.163:8554/stream 
    ![输入图片说明4](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/detect_and_classify/out1.png)

5.执行样例运行脚本

```
bash sample_run.sh
```