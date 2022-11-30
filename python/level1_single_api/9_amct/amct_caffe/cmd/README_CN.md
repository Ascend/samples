# Caffe 框架示例

Caffe 框架 ResNet-50 分类网络模型量化

## 1. 准备模型

通过执行以下命令下载原始模型定义文件 (.prototxt) 与预训练权重文件 (.caffemodel)：

```bash
python3.7.5 ./src/download_models.py --close_certificate_verify
```

> 其中，`--close_certificate_verify` 参数可选，用于关闭证书验证参数，确保模型正常下载。如果模型下载过程中提示认证失败相关信息，则可以增加该参数重新下载。

若执行成功，将会下载 `ResNet-50-deploy.prototxt` 与 `ResNet-50-model.caffemodel` 到该样例根目录的 [model](./model/) 路径下。

> 若用户环境无法连接网络,请先在可连通网络的服务器下载相应文件后上传到 [model](./model/) 路径下：
>
> ```bash
> wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-deploy.prototxt
> wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-model.caffemodel
> ```

## 2. 准备校准数据集

校准集用来产生量化因子，保证精度。计算量化参数的过程被称为“校准 (calibration)”。校准过程需要使用一部分图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。为了保证量化精度，校准集与测试精度的数据集来源应一致。

该示例提供了一组样例校准集用于量化校准，可切换到该样例根目录下执行以下命令获取校准数据与标签：

```bash
cd data
mkdir image && cd image
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/amct_acl/classification/calibration.rar
unrar e calibration.rar
```

> 如果环境中没有安装 unrar 工具，可以通过以下命令获取：
>
> ```bash
> sudo apt-get install unrar
> ```

## 3. 执行量化

切换到该样例根目录下，执行如下命令进行量化：

```bash
bash ./scripts/run_calibration.sh 
```


## 4. 量化结果

量化成功后，在当前目录生成如下文件：

+ [amct_log](./amct_log/)
  + [amct_caffe.log](./amct_log/amct_caffe.log): 量化日志文件，记录昇腾模型压缩工具量化过程的日志信息。
+ [results](./results/)
  + [resnet50__deploy_model.prototxt](./results/resnet50__deploy_model.prototxt): 量化后的部署模型文件。
  + [resnet50__deploy_weights.caffemodel](./results/resnet50__deploy_weights.caffemodel): 量化后的部署权重文件。
  + [resnet50__fake_quant_model.prototxt](./results/resnet50__fake_quant_model.prototxt): 量化后的仿真模型文件。
  + [resnet50__deploy_weights.caffemodel](./results/resnet50__deploy_weights.caffemodel): 量化后的仿真权重文件。
  + [resnet50__quant.json](./results/resnet50__quant.json): 量化算子信息。
