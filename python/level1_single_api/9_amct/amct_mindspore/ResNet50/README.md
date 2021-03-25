# ResNet-50 calibration sample

This is a sample to quick start with amct-mindspore tool

## Usage

### 1. Prepare the checkpoint

now you can download the pretrained ResNet-50 based on CIFAR-10 dataset from MindSpore official website;

[resnet50 mindspore pretrain model download page](https://www.mindspore.cn/resources/hub/details?MindSpore/ascend/0.7/resnet50_v1.5_cifar10)

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

