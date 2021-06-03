# faster-rcnn sample 的说明

## 1. 环境准备

如果使用 FasterRCNN 模型，则执行步骤环境初始化时会将模型自动下载到本地，本手册以该场景下的模型为例进行说明，用户也可以自行准备模型。
环境初始化用于获取检测网络源代码、模型文件、权重文件以及数据集等信息。

1. 获取检测网络源代码等信息。

   - 若安装昇腾模型压缩工具的服务器能连接网络，也能连通 GitHub。
   执行检测网络量化包 sample/faster_rcnn 中的如下脚本，初始化环境。

      ```none
      cd scripts && bash init_env.sh arg[1] arg[2] arg[3] arg[4] arg[5]
      ```

      参数解释如表 1 所示，命令使用示例如下，请根据实际情况进行替换：

      ```bash
      cd scripts && bash init_env.sh CPU **/caffe-master/ python3.7 /usr/include/python3.7m
      ```

   - 如果用户环境无法连接网络，则请在可连通网络的服务器，分别访问如下链接下载相应软件包，然后上传到 sample/faster_rcnn/scripts 目录。

   - [faster_rcnn 脚本路径](https://github.com/rbgirshick/caffe-fast-rcnn/archive/0dcd397b29507b8314e252e850518c5695efbb83.zip)：下载后重命名为 faster_rcnn_caffe_master.zip，环境初始化后生成 faster_rcnn Caffe 工程。
   - [faster_rcnn caffe_master 工程](https://github.com/rbgirshick/py-faster-rcnn/archive/master.zip)：下载后重命名为 py-faster-rcnn-master.zip，执行后生成 faster_rcnn 工程压缩包。
   - [vgg16_faster_rcnn 预训练模型文件](https://dl.dropboxusercontent.com/s/o6ii098bu51d139/faster_rcnn_models.tgz)：环境初始化后生成 vgg-16 faster_rcnn 预训练模型。
   - [VOC2007数据集](http://host.robots.ox.ac.uk/pascal/VOC/voc2007/VOCtrainval_06-Nov-2007.tar)：环境初始化后生成 VOC2007 数据集。该数据集只有在模型精度测试时才使用，详情请参见模型精度测试。

   切换到sample/faster_rcnn/scripts目录，执行初始化脚本，示例如下：

   ```none
   init_env.sh CPU **/caffe-master/ python3.7 /usr/include/python3.7m
   ```

   初始化脚本优先检查当前目录是否存在上述下载的软件包，如果已经存在，则不会连接网络再次下载，直接使用上述软件包中的内容生成相关目录下的文件。

   表 1 环境初始化脚本所用参数说明

   | 参数 | 说明 |
   | :-: | :-- |
   | [h] | 可选。显示帮助信息。 |
   | arg[1] | 必填。配置脚本运行模式，包括 CPU 或 GPU。说明： 若环境初始化时使用 CPU 参数，则量化时量化命令只能使用 [--cpu] 参数。 若环境初始化时使用 GPU 参数，则量化时量化命令可以使用 [--gpu GPU_ID] 或[--cpu] 参数。 用户根据实际情况选择环境初始化使用的参数。 |
   | arg[2] | 必填。caffe-master 的绝对路径。 |
   | arg[3] | 可选。配置 python3 版本，默认为 python3，用户存在多个版本时可以通过该参数指定 python3 版本，例如 python3.7。 |
   | arg[4] | 可选。配置 python3m 路径，需要同 python3 版本对应，默认为 `/usr/include/python3.7m`。 |
   | arg[5] | 可选。如果需要运行模型精度测试，需要配置为 with_benchmark，详情请参见模型精度测试。 |

2. 环境初始化完成后，会生成如下目录：

   - caffe_master_patch：Caffe 源代码替换文件，需要用户手动将如下文件复制到 caffe-master 工程下。
   - caffe_master_patch/include/caffe/fast_rcnn_layers.hpp：用于存放自定义层定义头文件。
   - caffe_master_patch/src/caffe：用于存放自定层实现源文件。
     命令如下：

      ```bash
      cp -r $HOME/amct/amct_caffe/sample/faster_rcnn/caffe_master_patch/* caffe-master/
      ```

3. src 中多出如下三个文件夹：
   - datasets：FasterRCNN 使用的数据集。

   - pre_model：FasterRCNN 模型文件（faster_rcnn_test.pt）以及权重文件（VGG16_faster_rcnn_final.caffemodel）。

   - python_tools：FasterRCNN 模型源代码。

完成后，返回 caffe-master 目录，分别执行如下命令重新编译 Caffe 环境。

   ```none
   make clean && make all && make pycaffe
   ```

## 2. 量化示例

### 2.1 对原始网络模型进行预测试

   量化前，需要先将原始模型和数据集在 Caffe 环境中执行推理过程，以避免数据集和模型不匹配、模型无法在 Caffe 环境中执行的问题。

   切换到 sample/faster_rcnn/src 目录，执行如下命令检测 faster_rcnn 网络模型。

   ```none
   python3.7.5 faster_rcnn_sample.py --model_file MODEL_FILE --weights_file WEIGHTS_FILE [--gpu GPU_ID] [--cpu][--iterations ITERATIONS] [--pre_test]
   ```

   量化脚本 faster_rcnn_sample.py 的参数说明：

   | 参数名 | 说明 |
   | :-- | :-- |
   | --model_file | 必选，模型输入的 prototxt 文件 |
   | --weights_file | 必选，模型权重 caffemodel 文件 |
   | --gpu | 可选，使用 GPU 进行推理则指定 gpu id |
   | --cpu | 可选，使用 CPU 进行推理 |
   | --iterations | 可选，指定进行推理的 batch 数，基于 sample 的简单数据集，这个数值不可以大于5 |
   | --pre_test | 可选，进行未量化模型的精度测试 |

   使用样例如下：

   ```none
   python3.7.5 faster_rcnn_sample.py --model_file pre_model/faster_rcnn_test.pt --weights_file pre_model/VGG16_faster_rcnn_final.caffemodel  --gpu 0 --pre_test
   ```

   根据 src/datasets 数据集中检测对象的数量，会展示相应数量的检测结果文件，关闭检测结果文件，若昇腾模型压缩工具所在服务器出现如下信息，则说明原始模型在 Caffe 环境中运行正常。

   ```none
   [AMCT][INFO]Run faster_rcnn without quantize success!
   ```

   预检测结果文件存放路径为 `src/pre_detect_results/`。

### 2.2 执行量化

   ```none
   python3.7.5 faster_rcnn_sample.py --model_file pre_model/faster_rcnn_test.pt --weights_file pre_model/VGG16_faster_rcnn_final.caffemodel  --gpu 0
   ```

   ​根据 src/datasets 数据集中检测对象的数量，展示相应数量的检测结果文件，您可以根据图片上检测框的位置和使用“[--pre_test]”参数后的原始模型的推理结果进行比较。

   将所有检测结果文件关闭，在昇腾模型压缩工具所在服务器还可以看到如下量化成功信息：

   ```none
   [AMCT][INFO]Run faster_rcnn with quantize success!
   ```

   量化后检测结果文件存放路径为 `src/quant_detect_results/`。

### 2.3 量化结果展示

量化成功后，界面会显示量化后精度仿真模型的推理结果。在量化后模型的同级目录下生成量化配置文件 config.json、量化日志文件夹 amct_log、量化结果文件results、量化中间结果文件 tmp 等：

- config.json：描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量    化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件与已有的    文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。实际量化过程中，如果量化后的模型推理精度不满足要求，用户可以修改 config.json 文件，    量化配置文件内容、修改原则以及参数解释请参见量化配置。
- amct_log：记录了工具的日志信息，包括量化过程的日志信息 amct_caffe.log。
- pre_detect_results：预检测结果文件存放路径。
- quant_detect_results：量化后检测结果文件存放路径。
- tmp：量化过程中产生的文件，包括中间模型文件 modified_model.prototxt、modified_model.caffemodel，记录量化因子的文件 scale_offset_record/record.txt（关于该文件的原型定义请参见量化因子记录文件说明）。
- results：量化结果文件，包括量化后的模型文件、权重文件以及模型量化信息文件，如下所示。
- faster_rcnn_deploy_model.prototxt：量化后的可在昇腾 AI 处理器部署的模型文件。
- faster_rcnn_deploy_weights.caffemodel：量化后的可在昇腾 AI 处理器部署的权重文件。
- faster_rcnn_fake_quant_model.prototxt：量化后的可在 Caffe 环境进行精度仿真模型文件。
- faster_rcnn_fake_quant_weights.caffemodel：量化后的可在 Caffe 环境进行精度仿真权重文件
- faster_rcnn_quant.json：量化信息文件（该文件名称和量化后模型名称保持统一），记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。

对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

（后续处理）如果用户需要将量化后的 deploy 模型，转换为适配昇腾AI处理器的离线模型，则请参见 ATC  工具的使用说明书。

## 3. 模型精度测试

由于量化示例进行的推理和量化校准的过程都是基于自带的图片数据集进行的，量化结果仅用于验证量化模型是否成功，不能够作为量化后模型精度验证标准。本章节给出基于 VOC2007 标准数据集进行量化前后网络精度验证测试的详细步骤。

在初始化环境时增加参数 with_benchmark，用于下载 VOC2007 标准数据集。

### 3.1 准备动作

执行如下命令初始化环境信息，用于下载 VOC2007 标准数据集。

```bash
cd scripts && bash init_env.sh CPU **/caffe-master with_benchmark
或
cd scripts && bash init_env.sh CPU **/caffe-master python3.7.5 /usr/include/python3.7m with_benchmark
```

环境初始化完成后，除了重新生成环境初始化中的文件外，还会额外在 src/datasets 目录生成 VOCdevkit 数据集文件。

如果环境初始化时，加了 with_benchmark 参数，则后续所有的量化动作都是基于 VOC2007 标准数据集操作的。

若环境初始化时使用CPU参数，则量化时量化命令只能使用 `--cpu` 参数。

若环境初始化时使用GPU参数，则量化时量化命令可以使用 `--gpu GPU_ID` 或 `--cpu` 参数。用户根据实际情况选择环境初始化使用的参数。

### 3.2 精度测试

1. 量化前精度测试。
   命令如下：

   ```bash
   cd src
   python3.7.5 faster_rcnn_sample.py --model_file pre_model/faster_rcnn_test.pt --weights_file pre_model/VGG16_faster_rcnn_final.caffemodel  --gpu 0 --pre_test
   ```

   参数解释请参见表 1，若出现如下信息则说明执行成功：

   ```none
   [AMCT][INFO]Run faster_rcnn without quantize success, and mAP is 0.8812724482290413
   ```

2. 量化后精度测试。

   ```bash
   cd src
   python3.7.5 faster_rcnn_sample.py --model_file pre_model/faster_rcnn_test.pt --weights_file pre_model/VGG16_faster_rcnn_final.caffemodel  --gpu 0
   ```

   若出现如下信息则说明量化成功（如下推理精度只是样例，请以实际环境量化结果为准）：

   ```none
   [AMCT][INFO]Run faster_rcnn with quantize success, and mAP is 0.8796338534980108!
   ```

3. 用户可以根据量化前后 mAP(mean average precision) 的取值，查看量化是否满足要求。
