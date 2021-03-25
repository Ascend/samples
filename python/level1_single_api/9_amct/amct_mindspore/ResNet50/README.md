# ResNet-50 calibration sample

this is a sample to quick start with amct-mindspore tool

## Usage

### 1. Prepare the checkpoint

now you can download the pretrained ResNet-50 based on CIFAR-10 dataset from MindSpore official website;

[Mindspore pretrain model download page](https://www.mindspore.cn/docs/zh-CN/r0.7/network_list.html)

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

### 3. Download the model file

run the download_files.py scripts to download the resnet50 model defination file from github;

```python
python3 download_files.py
```

### 4. Run the calibration script

```python
python3 resnet50_sample.py --dataset_path your dataset path  --checkpoint_path your resnet50 checkpoint file path
```

