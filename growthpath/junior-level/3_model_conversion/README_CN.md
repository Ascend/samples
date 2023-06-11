中文|[English](./README.md)
# 模型转换

### <a name="step_1"></a> 本章学习目标
- 了解模型转换的目的
- 掌握模型转换工具的使用方法及常用参数意义
- 了解AIPP并掌握AIPP的使用方法

### <a name="step_2"></a>模型转换工具介绍
对于开源框架的网络模型（如ONNX、TensorFlow等）或Ascend IR定义的单算子描述文件（json格式），不能直接在昇腾AI处理器上做推理，需要先使用模型转换工具ATC（Ascend Tensor Compiler）将开源框架的网络模型或单算子文件转换为适配昇腾AI处理器的离线模型（*.om文件）。    
模型转换过程中，ATC会进行算子调度优化、权重数据重排、内存使用优化等具体操作，对原始的深度学习模型进行进一步的调优，从而满足部署场景下的高性能需求，使其能够高效执行在昇腾AI处理器上。ATC工具的功能架构图如下所示：    
![ATC工具功能架构](https://www.hiascend.com/doc_center/source/zh/CANNCommunityEdition/600alpha003/infacldevg/atctool/figure/zh-cn_image_0000001467282525.png)  

其中：    
- 开源框架网络模型场景：    
  1. 开源框架网络模型经过Parser解析后，转换为中间态IR Graph。   
  2. 中间态IR经过图准备，图拆分，图优化，图编译等一系列操作后，转成适配昇腾AI处理器的离线模型。    
  3. 转换后的离线模型上传到板端环境，通过AscendCL接口加载模型文件实现推理过程。     
- 单算子描述文件场景下：    
  1. 通过ATC工具进行单算子编译后，转成适配昇腾AI处理器的单算子离线模型。   
  2. 用户可通过AscendCL接口加载单算子模型文件、执行单算子。
**本章节主要就使用ATC工具进行模型转换进行介绍**

### <a name="step_3"></a>运行流程
使用ATC工具进行模型转换的运行流程如下图所示。        
![AscendCL逻辑架构](https://www.hiascend.com/doc_center/source/zh/CANNCommunityEdition/600alpha003/infacldevg/atctool/figure/zh-cn_image_0000001417602960.png)    
1. 使用ATC工具之前，请先确保环境已安装Ascend-cann-toolkit包，并完成环境变量设置。
2. 准备要进行转换的模型或单算子描述文件，并上传到环境。
3. 使用ATC工具进行模型转换，在配置相关参数时，根据实际情况选择是否进行AIPP配置。

### <a name="step_4"></a>AIPP说明
- **什么是AIPP**    
   AIPP（Artificial Intelligence Pre-Processing）人工智能预处理，用于在AI Core上完成数据预处理，包括改变图像尺寸、色域转换（转换图像格式）、减均值/乘系数（改变图像像素），数据预处理之后再进行真正的模型推理；可以简单理解为使用AIPP后，会在模型原始结构的输入上再加上一层AIPP算子层，输入数据处理后，先经过AIPP的处理后再将结果送入模型。   
   AIPP有静态和动态两种方式，在初步学习过程中，我们仅需要了解静态AIPP的使用即可。   
   静态AIPP是指在模型转换时将AIPP需要的所有能力都配置到AIPP配置文件中进行模型转换，模型生成后，AIPP参数值被保存在离线模型中，每次模型推理过程采用固定的AIPP预处理参数进行处理，而且在之后的推理过程中无法通过业务代码进行直接的修改。   
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/mindstudio_resnet50om_visualization.png)
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/mindstudio_resnet50om_aipp_visualization.png)

- **如何使能AIPP**    
  以下配置文件及流程仅作示例，配置文件参数值及模型转换命令需要根据实际诉求进行配置。    
  1. 构造AIPP配置文件,如test.cfg    
     ```
     aipp_op {
       aipp_mode : static             # AIPP配置模式，这里为静态AIPP
       input_format : YUV420SP_U8     # 输入给AIPP的原始图片格式
       src_image_size_w : 250         # 输入给AIPP的原始图片宽高
       src_image_size_h : 250
       crop: true                     # 抠图开关，用于改变图片尺寸
       load_start_pos_h: 0            # 抠图起始位置水平、垂直方向坐标
       load_start_pos_w: 0
       csc_switch : true              # 色域转换开关
       matrix_r0c0 : 256              # 色域转换系数，用户无需修改
       matrix_r0c1 : 0
       matrix_r0c2 : 359
       matrix_r1c0 : 256
       matrix_r1c1 : -88
       matrix_r1c2 : -183
       matrix_r2c0 : 256
       matrix_r2c1 : 454
       matrix_r2c2 : 0
       input_bias_0 : 0
       input_bias_1 : 128
       input_bias_2 : 128
       mean_chn_0: 104                # 归一化配置
       mean_chn_1: 117
       mean_chn_2: 123
       min_chn_0: 0.0
       min_chn_1: 0.0
       min_chn_2: 0.0
       var_reci_chn_0: 1.0
       var_reci_chn_1: 1.0
       var_reci_chn_2: 1.0
     }
     ```   
  2. 使能静态AIPP，将其配置参数保存在模型文件中     
     ```   
     atc --framework=${framework} --soc_version=${soc_version} --model=${model} --insert_op_conf=./test.cfg --output=${output}
     ```
  3. 调用AscendCL接口加载模型，执行推理     
  
- **AIPP功能说明**    
  1. **Crop/Padding**    
     AIPP改变图片尺寸需要遵守如下图中的顺序，即先Crop再Padding，每个操作仅能执行一次。      
     ![Crop/Padding](https://www.hiascend.com/doc_center/source/zh/CANNCommunityEdition/600alpha003/infacldevg/atctool/figure/zh-cn_image_0000001467441889.png)    
     上图场景可以抽象成如下aipp的配置。        
     ```  
     aipp_op {
       aipp_mode: static
       input_format: YUV420SP_U8

       src_image_size_w: 320  
       src_image_size_h: 240

       crop: true
       load_start_pos_w: 10
       load_start_pos_h: 20
       crop_size_w: 50
       crop_size_h: 60

       padding: true
       left_padding_size: 20
       right_padding_size: 15
       top_padding_size: 20
       bottom_padding_size: 15
       padding_value: 0
     }  
     ```     
     该场景功能：输入到AIPP中的图片尺寸为srcImageSizeW, srcImageSizeH；模型要求的图片尺寸为dstImageSizeW、dstImageSizeH。   
     Crop操作：在原图中距离左上角宽load_start_pos_w，高load_start_pos_h距离的点裁剪出宽为crop_size_w，高为crop_size_h大小的图片。   
     Paste操作：指明在裁剪后的图像四周padding的尺寸，即left_padding_size、right_padding_size、top_padding_size和bottom_padding_size。而经过图像尺寸改变之后最终图片大小，需要跟模型文件输入的图像大小即模型要求的图片尺寸dstImageSizeW、dstImageSizeH相等。   
     其中，对于YUV420SP_U8图片类型，load_start_pos_w、load_start_pos_h参数必须配置为偶数。     

     实际上，大多图像缩放或裁剪的工作都在样例中的输入预处理中做过了，如使用opencv的resize函数进行缩放等。初步学习的过程中，仅需要了解AIPP具备这样的功能即可。   
     在test.cfg中，仅做了Crop操作，片段如下：     
     ```
     crop: true                     # 抠图开关，用于改变图片尺寸
     load_start_pos_h: 0            # 抠图起始位置水平、垂直方向坐标
     load_start_pos_w: 0
     ```
     开启Crop且没有配置padding，crop_size_w和crop_size_h才能取值为0或不配置。    
     此时抠图大小（crop_size[W|H]）的宽和高取值来自模型文件--input_shape中的宽和高，并且--input_shape中的宽和高取值范围为[1,4096]。    
     换言之，通过这样的简单配置，就可以自动将输入的图片裁剪成模型要求的图片大小，避免输入预处理后的图片与模型要求的图片大小不一致的问题。但可能会带来丢失一部分原始图片而导致的精度损失问题。    

  2. **色域转换**      
     色域转换功能由csc_switch参数控制，并通过色域转换系数matrix_r*c*、通道交换rbuv_swap_switch等参数配合使用。     
     AIPP提供了一个比较方便的功能，就是一旦确认了AIPP处理前与AIPP处理后的图片格式，即可确定色域转换相关的参数值，用户无需修改，即上述参数都可以直接从模板中进行复制，模板示例以及更多配置模板请参见[色域转换配置说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha003/infacldevg/atctool/atlasatc_16_0023.html)。   
     在test.cfg中，就做了一个YUV420SP转RGB的色域转换操作，片段如下：     
     ```
     csc_switch : true              # 色域转换开关
     matrix_r0c0 : 256              # 色域转换系数，用户无需修改
     matrix_r0c1 : 0
     matrix_r0c2 : 359
     matrix_r1c0 : 256
     matrix_r1c1 : -88
     matrix_r1c2 : -183
     matrix_r2c0 : 256
     matrix_r2c1 : 454
     matrix_r2c2 : 0
     input_bias_0 : 0
     input_bias_1 : 128
     input_bias_2 : 128     
     ```
  3. **归一化**   
     归一化就是要把需要处理的数据经过处理后限制在一定范围内，方便后面数据的处理。    
     AIPP支持的归一化设置，通过减均值和乘系数的操作完成，这样的能力不仅能用于常规的归一化，还能用于不同数据格式的转化。     
     比如在由unit8转为fp16时，其转换可以视作如下公式。其中，mean_chn_i表示每个通道的均值，min_chn_i表示每个通道的最小值，var_reci_chn表示每个通道方差的倒数，各通路的这三个值都是需要进行配置的参数。    
     ```  
     pixel_out_chx(i)=[pixel_in_chx(i)-mean_chn_i-min_chn_i]*var_reci_chn
     ```   
     在test.cfg中，mean_chn_i有取值，min_chn_i为0，var_reci_chn为1.0，所以只做了减均值的操作，片段如下：     
     ```
     mean_chn_0: 104                # 归一化配置
     mean_chn_1: 117
     mean_chn_2: 123
     min_chn_0: 0.0
     min_chn_1: 0.0
     min_chn_2: 0.0
     var_reci_chn_0: 1.0
     var_reci_chn_1: 1.0
     var_reci_chn_2: 1.0
     ```
     值得注意的是，min_chn_0对应哪一个通道，取决于色域转换的结果。     
     如test.cfg中，将YUV420SP转成了RGB，那么此时mean_chn_0、min_chn_0、var_reci_chn_0对应的就是R通道，以此类推。

- **AIPP部分约束说明**     
  - 由于硬件处理逻辑的限制，AIPP处理顺序为：通道交换（rbuv_swap_switch）>图像裁剪（crop ）> 色域转换（通道交换） > 数据减均值/归一化 > 图像边缘填充（padding）。    
  - AIPP当前输入图片的类型仅支持RAW和UINT8格式。    

### <a name="step_5"></a>基本场景及参数介绍
接下来将根据[图片分类应用](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)中的模型转换介绍实际使用场景。    
1. 场景说明     
   前面介绍过，该样例使用的是已经训练好的pytorch框架的ResNet-50开源模型。模型的基本信息如下所示：    
   - 输入数据：RGB格式、224*224分辨率的输入图片。
   - 输出数据：图片的类别标签及其对应置信度。
   - 归一化：需要进行减均值（[0.485,0.456,0.406]）及乘系数([0.229,0.224,0.225])操作

2. 不使用AIPP进行模型转换    
   该模型的转换命令如下：    
   ```
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
   atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_format=NCHW --input_shape="actual_input_1:1,3,224,224" --soc_version=Ascend310
   ```
   经过模型转换后，输出的om模型和原始模型输入输出数据一致。此时数据预处理需要将数据处理为RGB格式、缩放到224*224分辨率、数据进行归一化后才能送入om模型进行推理。 
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/convert_resnet50_onnx.png)   

3. 使用AIPP进行模型转换    
   AIPP配置文件如下：    
   ```
   aipp_op{
     aipp_mode:static                        
     input_format : RGB888_U8

     src_image_size_w : 256
     src_image_size_h : 256

     crop: true
     load_start_pos_h : 16
     load_start_pos_w : 16
     crop_size_w : 224
     crop_size_h: 224

     min_chn_0 : 123.675
     min_chn_1 : 116.28
     min_chn_2 : 103.53
     var_reci_chn_0: 0.0171247538316637
     var_reci_chn_1: 0.0175070028011204
     var_reci_chn_2: 0.0174291938997821
   }
   ```
   模型转换命令如下：     
   ```
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50_CV/aipp.cfg  
   atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_format=NCHW --input_shape="actual_input_1:1,3,224,224" --soc_version=Ascend310 --insert_op_conf=aipp.cfg
   ```   
   经过模型转换后，输出的om模型和原始模型输入不一致，输出数据一致。此时数据预处理需要将数据处理为RGB格式、缩放到224*224分辨率后即可送入om模型进行推理。此时om会先通过AIPP进行归一化，然后再继续推理。 
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/convert_resnet50_onnx_with_aipp.png)   
   
4. 参数说明    
   - --model           
     填写原始网络模型文件路径与文件名。当原始模型为Caffe框架时，需要和--weight参数配合使用。参数使用参考示例如下：         
     ```   
     # Caffe模型
     --model=$HOME/module/resnet50.prototxt --weight=$HOME/module/resnet50.caffemodel
     # MindSpore模型
     --model=resnet50.air
     # pb模型
     --model=resnet50.pb
     # onnx模型
     --model=resnet50.onnx
     ```     
   - --framework         
     填写原始网络模型文件路径与文件名，参数取值与使用参考示例如下：  
     ```
     # 取值为0时，即为Caffe框架网络模型，模型包括后缀为prototxt的模型文件和后缀为caffemodel的权重文件
     --framework=0
     # 取值为1时，即为MindSpore框架网络模型，仅支持后缀为*.air的模型文件。
     --framework=1
     # 取值为3时，即为TensorFlow框架网络模型，只支持尾缀为pb的模型文件
     --framework=3
     # 取值为5时，即为ONNX格式网络模型，仅支持ai.onnx算子域中opset v11版本的算子，用户也可以将其他opset版本的算子（比如opset v9），通过PyTorch转换成opset v11版本的ONNX算子；而使用PyTorch训练出的pth模型需要转化为ONNX格式的模型，才能进行模型转换。
     --framework=5
     ```
   - --input_format  
     填写输入数据格式，Caffe、ONNX默认为NCHW；TensorFlow默认为NHWC。   
     **该参数一般不需要填写，直接使用默认值即可**，但是该参数存在以下限制。    
     1. 如果TensorFlow模型是通过ONNX模型转换工具输出的，则该参数必填，且值为NCHW。  
     2. 当原始框架为MindSpore时，只支持配置为NCHW。  
     3. 如果模型转换时开启AIPP，在进行推理业务时，输入图片数据要求为NHWC排布，该场景下最终与AIPP连接的输入节点的格式被强制改成NHWC，可能与atc模型转换命令中--input_format参数指定的格式不一致。   
     ```
     # 参数填写示例如下
     --input_format=NCHW
     ```    
   - --input_shape    
     填写模型输入输出的shape，格式为"input_name:n,c,h,w"；模型输入唯一且shape固定时，可以不填写该参数。以下情况必须要设置该参数：   
     1. 模型有多个输入，则不同输入之间使用英文分号分隔，例如"input_name1:n1,c1,h1,w1;input_name2:n2,c2,h2,w2"。input_name必须是转换前的网络模型中的节点名称。
     2. 原始模型中输入数据的某个维度不固定，例如input_name1:？,h,w,c，该参数必填。其中“？”为batch数，表示一次处理的图片数量，可以直接设置为固定值。
     在本章节介绍的resnet50模型转换过程中，由于原始onnx模型输入为动态Batch的，所以必须要设置该参数，调整为单Batch。设置如下：   
     ```  
     --input_shape="actual_input_1:1,3,224,224"
     ```
     该参数的难点在于要了解原始模型，可以通过netron工具打开原始模型，找到输入节点后填写。  

     ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/netron_model_input_resnet50_onnx.png) 
     
   - --output    
     当进行开源框架的网络模型转换时，填写输出的离线模型的路径以及文件名。参数使用参考示例如下：   
     ```   
     # 指定目录
     --output=$HOME/resnet50
     # 当前目录
     --output=resnet50
     ```     
     换后的模型文件名以该参数指定的为准，自动以.om后缀结尾，以上示例最终生成的模型为resnet50.om。  
   - --soc_version     
     填写模型转换时昇腾AI处理器的版本。参数使用参考示例如下：   
     ```
     # 昇腾310 AI处理器使用示例
     --soc_version=Ascend310
     # 昇腾310P AI处理器使用示例
     --soc_version=Ascend310P3
     # 昇腾910 AI处理器使用示例
     --soc_version=Ascend910A
     ```
     本章节中所有示例都是以昇腾310 AI处理为例进行说明，如果使用其它芯片产品，请在模型转换时自行替换该参数取值。   
   - --insert_op_conf   
     填写插入算子的配置文件路径与文件名，例如aipp预处理算子。   
     本章节中主要使用场景为插入aipp预处理算子，使用该参数后，则输入数据类型为UINT8。参数使用参考示例如下：   
     ```
     --insert_op_conf=aipp.cfg
     ```
5. 扩展说明   
   结合以上说明，将每个基础参数进行组合，给出以下模型转换的示例，可以尝试在自己的环境中执行以下命令，体验使用不同参数进行模型转换的区别。    
   ```
   # caffe模型，不使用aipp
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt
   atc --model=resnet50.prototxt --weight=resnet50.caffemodel --framework=0 --output=resnet50 --soc_version=Ascend310 --input_format=NCHW

   # pb模型，不使用aipp
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/resnet50_tensorflow_1.7.pb
   atc --model=resnet50_tensorflow_1.7.pb --framework=3 --output=resnet50 --soc_version=Ascend310

   # onnx模型，不使用aipp
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
   atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_shape="actual_input_1:1,3,224,224" --soc_version=Ascend310
   
   # onnx模型，使用aipp
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50_CV/aipp.cfg
   atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_shape="actual_input_1:1,3,224,224" --soc_version=Ascend310 --insert_op_conf=aipp.cfg
   ```   
   也可以结合以上的模型文件，尝试增加aipp及研究其它参数。同时，转换完成的om模型可以通过MindStudio打开，可以打开对比下原始模型和转换后模型的差异。
### <a name="step_6"></a>其它场景及参数介绍
除了离线模型转换外，ATC工具还提供了很多其它场景，这些场景也有着特定的参数使用方法。    
    
1. 单算子模型转换      
   单算子描述文件是基于Ascend IR定义的单个算子的定义文件，包括算子的输入、输出及属性信息；借助该文件转换成适配昇腾AI处理器的离线模型后，可以验证单算子的功能。单算子模型转换示例如下：    
   ```
   mkdir op_model
   wget xxx/gemm.json
   atc --singleop=gemm.json --output=op_model --soc_version=Ascend310
   ```
   转换成功后，可以在--output制定的目录下，查看到生成的单算子离线模型文件。参数说明如下：     
   - --singleop： 填写单算子描述文件（*.json）路径。仅在单算子模型转换时使用。   
   - --output：和离线模型转换时不同的是，离线模型转换时该参数指定生成的离线模型名称，而单算子模型转换时仅指定模型路径。   
   - --soc_version：和离线模型转换时的使用方法完全一致。  
 
2. 将原始模型或离线模型转成json文件     
   如果用户不方便查看原始模型或离线模型的参数信息时，可以将原始模型或离线模型转成json文件进行查看。转换示例如下所示：    
   ```
   # 原始模型文件转json文件
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/resnet50_tensorflow_1.7.pb
   atc --mode=1 --om=resnet50_tensorflow_1.7.pb --json=tf_resnet50.json --framework=3

   # 离线模型文件转json文件
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/tf_resnet50.om
   atc --mode=1 --om=tf_resnet50.om --json=tf_resnet50.json
   ```
   将模型转换为json文件后，就可以查询到模型的详细信息，进而进行分析；

   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/tf_resnet50_json.png)

   参数分析如下：    
   - --mode：运行模式。离线模型转换时取值为默认值0，所以省略了；转json文件时设置为1，必须要填写该参数。
   - --om：离线模型、原始模型等文件的路径和文件名。
   - --json：离线模型、原始模型等文件转换为json格式文件的路径和文件名。仅在转json文件时使用。
   - --framework：和离线模型转换时的使用方法完全一致。

3. 查询离线om模型的详细信息    
   在进行一些模型问题时，可以先通过以下命令查询下om模型的详细信息，以便分析是不是原始模型就存在问题。    
   ```   
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/resnet50_imagenet_dynamic_hw-python/tf_resnet50.om
   atc --mode=6 --om=tf_resnet50.om
   ```
   命令执行完毕，屏幕会打印类似如下信息：    
   ```
   ============ Display Model Info start ============
   # 模型转换使用的atc命令
   Original Atc command line: ${INSTALL_DIR}/bin/atc.bin --model=XXX --framework=XXX --output=XXX --soc_version=XXX
   # ATC软件版本信息、soc_version版本信息、原始框架信息
   system   info: atc_version[xxx], soc_version[xxx], framework_type[xxx].
   # 运行时的占用内存、逻辑stream数目、event数目
   resource info: memory_size[xxx B], weight_size[xxx B], stream_num[xxx], event_num[xxx].
   # 离线模型文件中各分区大小、包括ModelDef、权重、tbekernels、taskinfo占用的大小等
   om       info: modeldef_size[xxx B], weight_data_size[xxx B], tbe_kernels_size[xxx B], cust_aicpu_kernel_store_size[xxx B], task_info_size[xxx B].
   ============ Display Model Info end   ============
   ```
   参数分析如下：   
   - --mode：运行模式。取非默认值6，代表针对已有的离线模型，显示模型信息，必须要填写该参数。
   - --om：离线模型、原始模型等文件的路径和文件名。

### <a name="step_7"></a>样例及场景介绍
以下提供了两个场景的样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [sampleResnetQuickStart](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)  | 使用opencv进行预处理，模型转换时不带AIPP的Resnet50分类样例  |
| [sampleResnetAIPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)    | 使用opencv进行预处理，模型转换时使用AIPP的Resnet50分类样例  |