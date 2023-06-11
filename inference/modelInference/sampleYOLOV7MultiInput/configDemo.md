## JSON语法

 JSON基本语法如下：

- 数组(Array)用方括号 "[]" 表示。
- 对象(0bject)用大括号 "{}" 表示。
- 名称/值 对(name/value)组合成数组和对象。
- 名称(name)置于双引号中，值(value)有字符串、数值、布尔值、null、对象和数组。
- 并列的数据之间用逗号 "," 分隔。
- 名称/值对包括字段名称(在双引号中)，后面写一个冒号，然后是值。

 需要注意的是：

 JSON不支持注释。向 JSON添加注释无效；

 JSON文件的文件类型是 .json；

 JSON文本的MIME类型是 application/json；

## 输入图片，输出图片的配置流程

1. 下载获取输入图片，放在data目录下
```
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg --no-check-certificate
```

2. 修改配置文件中对应的input_path、input_type、output_path、output_type
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data",
                            "input_type":"pic",
                            "output_path":"",
                            "output_type":"pic",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

3.执行样例运行脚本

```
bash sample_run.sh
```

## 输入MP4视频文件，输出保存离线MP4视频文件的配置流程

1. 下载获取输入视频，放在data目录下
```
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
```

2. 修改配置文件中对应的input_path、input_type、output_path、output_type
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"../out/output_car0.mp4",
                            "output_type":"video",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

3.执行样例运行脚本

```
bash sample_run.sh
```

##  输入rtsp流，输出打屏显示的配置流程（使用live555创建rtsp流，作为样例的输入数据）

1. 安装live555
```
# 下载live555
wget http://www.live555.com/liveMedia/public/live.2023.05.10.tar.gz
tar -zxvf live.2023.05.10.tar.gz
cd live/
./genMakefiles linux
make -j8
```

2. 数据准备

重新打开一个窗口进入/live/testProgs目录
```
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleResnetRtsp/test.264 --no-check-certificate
```

获取rtsp流地址

```
./testOnDemandRTSPServer
```
即可创建rtsp流如下图所示

![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleResnetRtsp/rtsp.jpg "image-20211028101534905.png")

获取rtsp流地址如上图黄色方框所示，例如 rtsp://192.168.0.163:8554/h264ESVideoTest

3. 修改配置文件中对应的input_path、input_type、output_path、output_type
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"rtsp://192.168.0.163:8554/h264ESVideoTest",
                            "input_type":"rtsp",
                            "output_path":"",
                            "output_type":"stdout",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```


## 输入MP4视频文件，输出打屏显示的配置流程

1. 下载获取输入视频，放在data目录下
```
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
```

2. 修改配置文件中对应的input_path、input_type、output_path、output_type
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"",
                            "output_type":"stdout",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

3.执行样例运行脚本

```
bash sample_run.sh
```

## 输入MP4视频文件，输出imshow窗口显示的配置流程

1. 下载获取输入视频，放在data目录下
```
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
```

2. 修改配置文件中对应的input_path、input_type、output_path、output_type
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"",
                            "output_type":"imshow",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

3.执行样例运行脚本

```
bash sample_run.sh
```

## 输入MP4视频文件，输出rtsp流显示的配置流程

1. 下载获取输入视频，放在data目录下
```
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
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

3. 修改配置文件中对应的input_path、input_type、output_path、output_type

```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
# 执行如下命令获取推流ip地址
ifconfig
#打开配置文件test.json根据ip地址进行修改
```
![输入图片说明2](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/ip%E6%88%AA%E5%9B%BE%E7%A4%BA%E4%BE%8B.png)
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"rtsp://192.168.1.214:8554/stream",
                            "output_type":"rtsp",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
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
    ![输入图片说明3](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/smplayer%E5%90%AF%E5%8A%A8%E6%88%AA%E5%9B%BE%E7%A4%BA%E4%BE%8B.png)
    smplayer窗口右键open>URL,输入URL地址：rtsp://192.168.1.214:8554/stream0 （URL地址由output_path+channel_id组成）
    ![输入图片说明4](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/URL%E5%9C%B0%E5%9D%80%E6%88%AA%E5%9B%BE%E7%A4%BA%E4%BE%8B.png)

5.执行样例运行脚本

```
bash sample_run.sh
```
在smplayer窗口中查看输出流信息

## 模型多batch场景替换
1. 将模型转为多batch：

以样例使用模型为例，进行ATC转换时，修改参数选项--input_shape输入shape的batch数为4：
```
atc --model=yolov7x.onnx --framework=5 --output=yolov7x --input_shape="images:4,3,640,640"  --soc_version=Ascend310  --insert_op_conf=aipp.cfg
```

2.修改配置文件中对应的model_batch
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":4,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"",
                            "output_type":"stdout",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

3.执行样例运行脚本

```
bash sample_run.sh
```

## 模型单路输入对应多后处理场景

对于单路数据输入，当前代码分为了dataInput、detectPreprocess、detectInference、detectPostprocess、dataOutput五个部分并行执行，性能瓶颈就会出现在这五个部分中耗时最长的线程里，而dataInput、detectPreprocess使用dvpp硬件加速，占用时长一般不会成为样例的运行瓶颈，而detectInference、detectPostprocess的运行时长就是我们优化样例的主力修改点了，当detectPostprocess远大于detectInference运行耗时时，增加后处理的个数，可以一定程度上优化样例性能。

以样例使用模型为例，通过打印各线程process方法耗时分析修改test.json文件中yolov7x模型对应的后处理个数：
分析：通过打印每个阶段处理一帧的耗时，可以得到当前样例在310P上的detectInference每帧处理时长约为12ms，detectPostprocess每帧处理时长约为24ms，此时得到后处理个数为2时，样例整体运行耗时最短。故修改test.json如下，postnum数设置为2：

1.修改配置文件中对应的postnum
```
# 切换到配置文件所在目录
cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
#打开配置文件test.json进行修改
```
```
{
    "device_config":[
        {
            "device_id":0,
            "model_config":[
                {
                    "infer_thread_name":"infer_thread_0",
                    "model_path":"../model/yolov7x.om",
                    "model_width":640,
                    "model_heigth":640,
                    "model_batch":1,
                    "postnum":2,
                    "io_info":[
                        {
                            "input_path":"../data/car0.mp4",
                            "input_type":"video",
                            "output_path":"",
                            "output_type":"stdout",
                            "channel_id":0
                        }
                    ]
                }
            ]
        }
    ]
}
```

2.执行样例运行脚本

```
bash sample_run.sh
```

## test.json配置参数详解

下面以[test.json](./scripts/test.json)文件为例，给大家展示如何配置该json文件使得案例成功运行。

```
{
    # device_config:device层面上的配置参数，后接[……]中的内容即是device_config的具体信息
    "device_config":[    
        {
            # device_id：当前device对象对应的deviceID值，由用户指定，需要给当前运行设备上存在的deviceID
            "device_id":0,
            # model_config: 单个模型推理层面上的配置参数，后接[……]中的内容即是model_config的具体信息
            "model_config":[
                {
                    # infer_thread_name：当前加载一路推理线程时所给到的对应的推理线程名称，由用户指定，infer_thread_name值必须互斥
                    "infer_thread_name":"infer_thread_0",
                    # model_path：当前推理线程所加载的离线模型路径，由用户指定
                    "model_path":"../model/yolov7x.om",
                    # model_width：当前推理线程所加载的离线模型的宽，由用户指定
                    "model_width":640,
                    # model_heigth：当前推理线程所加载的离线模型的高，由用户指定
                    "model_heigth":640,
                    # model_batch：当前推理线程所加载的离线模型的batch数，由用户指定
                    "model_batch":1,
                    # postnum：当前推理线程对应的后处理线程个数，由用户指定，模型推理线程时长：后处理线程时长<1:1时，postnum在(后处理线程时长/模型推理线程时长)左右时推理性能较优
                    "postnum":2,
                    # io_info：单个模型输入层面上的配置参数，后接[……]中的内容即是io_info的具体信息，每个{……}代表该模型的一路输入流信息
                    "io_info":[
                        {
                            # input_path：该模型的第一路输入流路径信息，input_type为video时，填入输入MP4离线视频路径，由用户指定
                            "input_path":"../data/car0.mp4",
                            # input_type：该模型的第一路输入流类型信息，由用户指定
                            "input_type":"video",
                            # output_path：该模型的第一路输入流对应输出路径，output_type为video时，填入输出MP4离线视频路径，由用户指定
                            "output_path":"../out/out0.mp4",
                            # output_type：该模型的第一路输入流对应输出类型信息，由用户指定
                            "output_type":"video",
                            # channel_id：表示该路输入流的通道ID，由用户指定，channel_id值必须互斥
                            "channel_id":0
                        }，
                        {
                            # input_path：该模型的第二路输入流路径信息，input_type为pic时，填入输入图片文件夹路径，由用户指定
                            "input_path":"../data",
                            # input_type：该模型的第二路输入流类型信息，由用户指定
                            "input_type":"pic",
                            # output_path：该模型的第二路输入流对应输出路径，output_type为pic时，值无效，可为空
                            "output_path":"",
                            # output_type：该模型的第二路输入流对应输出类型信息，由用户指定
                            "output_type":"pic",
                            # channel_id：表示该路输入流的通道ID，由用户指定，channel_id值必须互斥
                            "channel_id":1
                        }，
                        {
                            # input_path：该模型的第三路输入流路径信息，input_type为rtsp时，填入输入rtsp视频流地址，由用户指定
                            "input_path":"rtsp://192.168.1.214:8554/h264ESVideoTest",
                            # input_type：该模型的第三路输入流类型信息，由用户指定
                            "input_type":"rtsp",
                            # output_path：该模型的第三路输入流对应输出路径，output_type为stdout时，值无效，可为空
                            "output_path":"",
                            # output_type：该模型的第三路输入流对应输出类型信息，由用户指定
                            "output_type":"stdout",
                            # channel_id：表示该路输入流的通道ID，由用户指定，channel_id值必须互斥
                            "channel_id":2
                        }，
                        {
                            # input_path：该模型的第四路输入流路径信息，input_type为video时，填入输入MP4离线视频路径，由用户指定
                            "input_path":"../data/car1.mp4",
                            # input_type：该模型的第四路输入流类型信息，由用户指定
                            "input_type":"video",
                            # output_path：该模型的第四路输入流对应输出路径，output_type为rtsp时，填入输出rtsp视频流地址，由用户指定
                            "output_path":"rtsp://192.168.140.15:8554/stream",
                            # output_type：该模型的第四路输入流对应输出类型信息，由用户指定
                            "output_type":"rtsp",
                            # channel_id：表示该路输入流的通道ID，由用户指定，channel_id值必须互斥
                            "channel_id":3
                        }，
                        {
                            # input_path：该模型的第五路输入流路径信息，input_type为video时，填入输入MP4离线视频路径，由用户指定
                            "input_path":"../data/car2.mp4",
                            # input_type：该模型的第五路输入流类型信息，由用户指定
                            "input_type":"video",
                            # output_path：该模型的第五路输入流对应输出路径，output_type为imshow时，值无效，可为空
                            "output_path":"",
                            # output_type：该模型的第五路输入流对应输出类型信息，由用户指定
                            "output_type":"imshow",
                            # channel_id：表示该路输入流的通道ID，由用户指定，channel_id值必须互斥
                            "channel_id":4
                        }
                    ]
                }
            ]
        }
    ]
}
```