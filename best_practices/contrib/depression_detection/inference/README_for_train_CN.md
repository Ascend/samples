# 1. 模型训练指导

## 1.1. 环境说明

```
CANN 6.0.RC1
tensorflow==2.6.0
Scikit-learn==0.24.0
numpy==1.19.5
pandas==1.1.5
keras==2.6.0
```

 **说明：**

- 训练环境：Ubuntu20.04，CPU。

- 需安装openSMLIE工具。

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

## 1.2. 数据集处理

### 1.2.1. 数据集获取

> 该模型使用MODMA数据集（app重新采样）。

```
# 为了方便下载，在这里直接给出数据集下载命令,可以直接拷贝执行
cd ${HOME}/samples/best_practices/contrib/depression_detection/inference
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/train_audio_data.rar
unrar e train_audio_data.rar
```

### 1.2.2. 数据集预处理

​	执行预处理脚本preprocess.py，生成数据集预处理后的txt文件

```
python3 preprocess.py –-smile_file ${smile_file_path} –-config_file ${config_file_path} –audio_path ${audio_path} –output_path ${output_path}

示例：
python3.6 preprocess.py --smile_file /home/HwHiAiUser/opensmile/build/progsrc/smilextract/SMILExtract --config_file /home/HwHiAiUser/opensmile/config/emobase/emobase2010.conf --audio_path ./data/ --output_path ./dataout/
```

参数说明：

--smile_file：为opensmile工具SMILExtract文件路径。

--config_file：为opensmile工具emobase2010.conf文件路径。

--audio_path：音频文件路径。

--output_path：输出保存路径。

### 1.2.3. 数据集特征文件存放结构

```
|--audio_feature
  |--train
  	|--audio_feature01.txt
  	|--audio_feature02.txt
  	......
  	|--audio_feature38.txt
  |--test
  	|--audio_feature01.txt
  	|--audio_feature02.txt
  	......
  	|--audio_feature13.txt
```

## 1.3. 模型训练

执行CNN.py脚本，训练模型：

```
python3 CNN.py --feature_dir ${feature_dir} --output_path ${output_path}

示例：
python3 CNN.py --feature_dir ${HOME}/samples/best_practices/contrib/depression_detection/inference/audio_feature --output_path ${HOME}/samples/best_practices/contrib/depression_detection/inference/final_model
```

## 1.4. 结果

```
[ 2 24 26 14 29  1  3  8  9 13 15 18 25  5  6 11 19 27 28  7 23  4 10 17 12 22 21 16 20]  #模型特异性
[ 3 23  2 24 26  8  9 13 15 18 25  4 10 17 16 14 29  5  6 11 19 27 12 22 20  1 28 21  7]  #模型灵敏度
#测试集结果（前六个为抑郁症患者）
[1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
#准确率
acc：1.0
```
