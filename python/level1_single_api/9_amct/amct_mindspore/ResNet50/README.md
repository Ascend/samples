# ResNet-50 calibration and quantization aware training sample

amct_mindspore 工具 sample 运行指导

### 环境要求
1. Ascend 910 host 环境；
2. 安装 mindspore 1.2.0 Ascend 版本；
   安装指导参考 [mindspore 官网安装指导页](https://www.mindspore.cn/install)
3. 安装与 mindspore 1.2.0 配套的 amct_mindspore 1.0.7 版本；

## Usage

### 1. Prepare the checkpoint

你可以在mindspore的官网下载 ResNet50 的预训练 checkpoint 文件，训练基于 CIFAR-10 dataset；

[resnet50 mindspore pretrain model download page](https://www.mindspore.cn/resources/hub/details?MindSpore/ascend/0.7/resnet50_v1.5_cifar10)

将 resnet50.ckpt 文件放到 model 目录；

### 2. Prepare the dataset CIFAR-10

Dataset used: [CIFAR-10](http://www.cs.toronto.edu/~kriz/cifar.html)

- Dataset size：60,000 32*32 colorful images in 10 classes
  - Train：50,000 images
  - Test： 10,000 images
- Data format：binary files
  - Note：Data will be processed in dataset.py
- Download the dataset, the directory structure is as follows:

```
├─cifar-10-batches-bin
│
└─cifar-10-verify-bin
```

### 3. Download the model defination file

run the download_files.py scripts to download the resnet50 model defination file from github;

```python
cd scripts && python3 download_files.py
```

### 4. Run the calibration script

```python
cd src
python3 resnet50_sample.py --dataset_path your_dataset_path  --checkpoint_path your_resnet50_checkpoint_file_path
```

### 5. Run the quant aware training scritp
```python
cd src
python3 resnet50_retrain_sample.py 
```