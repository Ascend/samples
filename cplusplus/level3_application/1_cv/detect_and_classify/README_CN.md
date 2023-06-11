**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

# detection_and_classify样例
功能：使用 yolov3和 color模型对输入进行预测推理，并将结果打印到输出上。   
样例输入：原始图片jpg文件/视频mp4文件。   
样例输出：带推理结果的图片/视频文件。 

## 目录结构

```
├── model                      //模型文件夹，存放样例运行需要的模型文件
│   └── xxxx.pb                 
├── data                       //数据文件夹
│   └── xxxx                   //测试数据,输入图片/视频 
├── inc                        //头文件文件夹
│   └── CarParams.h            //声明样例使用的数据结构的头文件 
├── out                        //编译输出文件夹，存放编译生成的可执行文件
│   ├── xxxx                   //可执行文件 
│   └── output                 //结果输出文件夹（如果不存在需要自行创建）
│       └── xxxx               //样例运行的输出结果
├── display                    //网页展示功能实现代码文件夹
│   ├── presenterserver        //presenterserver文件夹
│   └── run_presenter_server.sh//presenterserver启动脚本
├── scripts                    //配置文件+脚本文件夹
│   ├── params.conf            //样例运行配置文件 
│   ├── present_start.conf     //presentserver启动配置文件 
│   ├── sample_build.sh        //快速编译脚本
│   └── sample_run.sh          //快速运行脚本
├── src 
│   ├── acl.json               //系统初始化的配置文件 
│   ├── CMakeLists.txt         //Cmake编译文件
│   ├── classifyPostprocess    //分类模型后处理线程文件夹，存放该业务线程的头文件及源码
│   ├── classifyPreprocess     //分类模型预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPostprocess      //检测模型后处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPreprocess       //检测模型预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── inference              //预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── presentagentDisplay    //网页展示线程文件夹，存放该业务线程的头文件及源码
│   └── main.cpp               //主函数，图片分类功能的实现文件  
└── CMakeLists.txt             //编译脚本入口，调用src目录下的CMakeLists文件
```

## 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv, ffmpeg+acllite | 请参考[第三方依赖安装指导(C++样例)](../../../environment)完成对应安装 |

## 样例准备
1. 准备源码包

可以使用以下两种方式下载，请选择其中一种进行源码准备。   
- 命令行方式下载（下载时间较长，但步骤简单）。
   ```    
   # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
   cd ${HOME}     
   git clone https://github.com/Ascend/samples.git
   ```
   **注：如果需要切换到其它tag版本，以v0.5.0为例，可执行以下命令。**
   ```
   git checkout v0.5.0
   ```
- 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
   **注：如果需要下载其它版本代码，请先请根据前置条件说明进行samples仓分支切换。**   
   ``` 
    # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
    # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
    # 3. 开发环境中，执行以下命令，解压zip包。     
    cd ${HOME}    
    unzip ascend-samples-master.zip

2. 准备模型和数据

### 准备模型

| **模型名称** | **模型说明**                                      | **模型详细描述**                                             |
| ------------ | ------------------------------------------------- | ------------------------------------------------------------ |
| yolov3       | 图片检测推理模型。是基于onnx的Yolov3模型。        | 模型详细描述请参见[https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov/ATC_yolov3_onnx_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov/ATC_yolov3_onnx_AE)。您可以参见readme中的“原始模型”章节下载原始模型网络文件和配置文件，也可以直接参见下方的[模型转换](#model_convert)章节使用wget命令下载。 |
| carcolor        | 车辆颜色分类推理模型。是基于tensorflow的CNN模型。 | 模型详细描述请参见[https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/ATC_CarColor_tensorflow_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/ATC_CarColor_tensorflow_AE)。您可以参见readme中的“原始模型”章节下载原始模型网络文件和配置文件，也可以直接参见下方的[模型转换](#model_convert)章节使用wget命令下载。 |

#### <a name="model_convert">模型转换</a>

需要将下载的原始模型转换为适配昇腾AI处理器的离线om模型，并放置到样例代码中的“model”目录下。

为方便操作，此处直接给出了原始模型的下载命令以及模型转换命令，可直接拷贝执行。当然，您也可以参见上方模型下载链接中的 Readme进行手工操作，并了解更多细节。

```
# 进入目标识别样例工程根目录
cd $HOME/samples/cplusplus/level3_application/1_cv/detect_and_classify
# 创建并进入model目录
mkdir model
cd model
# 下载yolov3的原始模型文件及AIPP配置文件
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/yolov3_t.onnx --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/aipp_onnx.cfg --no-check-certificate
# 执行模型转换命令，生成yolov3的适配昇腾AI处理器的离线模型文件
atc --model=./yolov3_t.onnx --framework=5 --output=yolov3 --input_shape="images:1,3,416,416;img_info:1,4" --soc_version=Ascend310 --input_fp16_nodes="img_info" --insert_op_conf=aipp_onnx.cfg
# 下载color模型的原始模型文件及AIPP配置文件
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/color.pb --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/aipp.cfg --no-check-certificate
# 执行模型转换命令，生成color的适配昇腾AI处理器的离线模型文件
atc --input_shape="input_1:-1,224,224,3" --output=./color_dynamic_batch --soc_version=Ascend310 --framework=3 --model=./color.pb --insert_op_conf=./aipp.cfg --dynamic_batch_size="1,2,4,8"
```

### 准备数据

样例编译时会自动下载测试数据，无需手工下载。

若您想自行下载测试数据，可参见如下命令：

```
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.mp4 --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.jpg --no-check-certificate
```

样例数据下载完后请存储在样例工程的data目录下。

## 编译运行
1. 在通用目标识别样例工程的根目录下执行以下命令，进行样例编译。

   ```
   cd scripts 
   bash sample_build.sh

   ``` 
   编译完成后，会在out文件夹下生成可执行文件main。

   1.1.当使用源码编译acl库时需要修改scripts下的sample_build.sh文件以及修改src目录下的CMakeLists.txt文件中acl源码以及头文件的路径,例子如下  

   sample_build.sh第40行

   ```  
   cmake ../../../src -DCMAKE_CXX_COMPILER=${TargetCompiler} -DCMAKE_SKIP_RPATH=TRUE -DUSE_LIBRARY=OFF -DUSE_PRESENT=ON   
   #修改-DUSE_LIBRARY=OFF,代表不使用acl动态库的方式(即源码编译)调用acl
   ```  

   CMakeLists.txt第57行-61行

   ```
   if(NOT USE_LIBRARY)
   include_directories(
           $ENV{HOME}/samples/cplusplus/common/acllite/include/  
           #将此路径更改为您acl库头文件的路径       
   )
   else()
   ```

   CMakeLists.txt第89行-94行

   ```
   if(NOT USE_LIBRARY)
   aux_source_directory($ENV{HOME}/samples/cplusplus/common/acllite/src/ aclLite)
   #将此路径更改为您acl库源码cpp文件的路径
   target_sources(main 
       PUBLIC
           ${aclLite})
   endif()
   ```

   1.2.如果未安装presentagent依赖，需要修改scripts下sample_build.sh文件

   sample_build.sh第40行
　 
   ```
   cmake ../../../src -DCMAKE_CXX_COMPILER=${TargetCompiler} -DCMAKE_SKIP_RPATH=TRUE -DUSE_LIBRARY=ON -DUSE_PRESENT=OFF   
   #修改-DUSE_PRESENT=OFF,代表不使用presentagent依赖
   ```

2. 修改scripts目录下的params.conf文件，配置样例的输入数据类型及结果展示类型。

   ```
   [base_options]
   device_num=1    # Device数量    
   RtspNumPerDevice=1      # 每个Device上的输入路数
   
   [rtsp_options_param] #rtsp的推拉流url
   URL=rtsp://192.168.220.175:8554/stream # 192.168.220.175为本地IP
   
   [options_param_0]    # 第1路的配置参数
   inputType_0=pic       # 第1路的输入数据类型
   outputType_0=pic    # 第1路的输出数据类型
   inputDataPath_0=../data/pic   # 第1路的输入数据路径
   
   #outputFrameWidth_0=1280  # outputType_0为video时，需要配置此参数，代表输出视频的宽
   #outputFrameHeight_0=720  # outputType_0为video时，需要配置此参数，代表输出视频的高
   
   #[options_param_1]    # 第2路的配置参数
   #inputType_1=video
   #outputType_1=presentagent
   #inputDataPath_1=../data/car2.mp4
   #outputFrameWidth_1=2368
   #outputFrameHeight_1=1080
   
   .......
   ```
   具体参数说明可参考[通用目标识别样例/样例编译运行](https://github.com/Ascend/samples/wikis/%E9%80%9A%E7%94%A8%E7%9B%AE%E6%A0%87%E8%AF%86%E5%88%AB%E6%A0%B7%E4%BE%8B/%E6%A0%B7%E4%BE%8B%E7%BC%96%E8%AF%91%E8%BF%90%E8%A1%8C)。

3. 若输出类型配置的为“presentagent”，运行可执行文件前可以参考[通用目标识别样例/样例编译运行](https://github.com/Ascend/samples/wikis/%E9%80%9A%E7%94%A8%E7%9B%AE%E6%A0%87%E8%AF%86%E5%88%AB%E6%A0%B7%E4%BE%8B/%E6%A0%B7%E4%BE%8B%E7%BC%96%E8%AF%91%E8%BF%90%E8%A1%8C)中相关内容进行 PresentServer的配置、启动和访问。若配置的其他输出类型，则此步骤可跳过。

  4. 若输出类型配置的为“rtsp”，则需要先安装rtsp-simple-server，然后启动rtsp服务，再运行程序，具体可参考[样例参数说明](./configDemo.md)中相关内容。若配置的其他输出类型，则此步骤可跳过。
 
5. 运行样例。

   ```
   cd ../out
   ./main
   ```

6. 查看运行结果。

   样例将根据配置的输出数据类型，输出不同文件：

   - 若输出数据类型配置为pic
     输出数据存储在out/output文件夹下，为名称类似于**device_X_out_pic_Y.jpg** 的图片，其中X代表第x路，Y代表第y张图片。

   - 若输出数据类型配置为video
     输出数据存储在out/output文件夹下，为名称类似于：**out_testX.mp4** 的视频，其中X代表第x路。

   - 若输出数据类型配置为presentagent
     请参见[访问PresentServer展示界面](#start_presentserver)查看推理结果。


 **更多样例详情可参考 [通用目标识别样例](https://github.com/Ascend/samples/wikis/%E9%80%9A%E7%94%A8%E7%9B%AE%E6%A0%87%E8%AF%86%E5%88%AB%E6%A0%B7%E4%BE%8B/%E5%89%8D%E8%A8%80/%E6%A6%82%E8%BF%B0)，包括如何进行定制开发和性能提升。** 