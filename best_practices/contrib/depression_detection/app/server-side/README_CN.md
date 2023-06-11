# 部署云服务器

## 1. 准备云服务器

1. 购买华为云服务器 ecs 弹性云服务器（昇腾310）。

2. 映射端口，添加入站规则，开放服务器8888端口，用于请求接口调用。

    说明：也可使用200DK（或者其他NPU服务器）结合局域网的形式搭建服务器端。


## 2. 环境说明


```
CANN 6.0.RC1
python == 3.6.8
django == 3.1.2
django-sslserver == 0.22
```

 **说明：**

- 需安装ffmpeg、openSMLIE工具：

  步骤 1   安装ffmpeg工具。

  ```
  sudo add-apt-repository universe
  sudo apt update
  sudo apt install ffmpeg

  验证是否安装成功：

  Ffmpeg

  出现版本号等信息代表安装成功
  ```
   步骤 2   安装openSMILE工具。
  ```
  git clone https://github.com/audeering/opensmile
  ```
  
  进入项目目录，执行如下命令，成功之后会生成build目录：
  ```
  cd opensmile

  sh build.sh
  ```
  将./build/progsrc/smilextract加入系统目录：
  ```
  vim /etc/profile
  ```
  在文档末尾添加：
  ```
  Export PATH=”/data/opensmile-master/build/progsrc/smilextract:$PATH”
  ```
  保存退出，然后执行：
  ```
  source /etc/profile
  ```
  测试安装是否成功。
  ```
  SMILExtract -h
  ```
- 下载acllite工具，在samples/python/common/acllite中。



## 3. 模型上传和转换

### 3.1. onnx模型上传

1. 将h5模型上传至/static/models文件夹中。

```
# 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。
cd ${HOME}/samples/best_practices/contrib/depression_detection/app/server-side/static/models
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_1.onnx
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_2.onnx
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_3.onnx
```


### 3.2. 模型转换（如果模型还是h5文件，则需要进行转换）

1. 执行h52onnx.py脚本，生成onnx模型文件

```
python3 h52onnx.py –h5_path ${h5_path} –onnx_path ${onnx_path}
```

2. 设置环境变量

```
source /usr/local/Ascend/ascend-toolkit/set_env.sh
```

该命令中使用CANN默认安装路径(/usr/local/Ascend/ascend-toolkit)中的环境变量，使用过程中请按照实际安装路径设置环境变量。

3. 使用atc将onnx模型转换为om模型文件，工具使用方法可以参考[CANN V100R020C10 开发辅助工具指南 (推理) 01](https://support.huawei.com/enterprise/zh/doc/EDOC1100164868?idPath=23710424%7C251366513%7C22892968%7C251168373)

   ${chip_name}可通过`npu-smi info`指令查看，例：310P3

![Image](https://github.com/Ascend/ModelZoo-PyTorch/raw/master/ACL_PyTorch/images/310P3.png)

三个om模型，对应的生成命令为：

```
cd ${HOME}/samples/best_practices/contrib/depression_detection/app/server-side/static/models
atc --framework=5 --model=model_1.onnx --output=tf_model_1 --input_format=NCHW --input_shape="conv2d_4_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}

atc --framework=5 --model=model_2.onnx --output=tf_model_2 --input_format=NCHW --input_shape="conv2d_8_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}

atc --framework=5 --model=model_3.onnx --output=tf_model_3 --input_format=NCHW --input_shape="conv2d_88_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}
```

## 4. 更改文件部分地址

final_test.py修改部分地址：

第19行pathExcuteFile为opensmile文件下的SMILExtract文件路径。

第21行pathConfig为opensmile配置文件emobase2010.conf所在的路径。

第126行为所用到的om模型所在路径。

运行在线推理web服务。

```
python3 manage.py runserver 0.0.0.0:8888
```
