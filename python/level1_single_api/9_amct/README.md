English|[中文](README_CN.md)

# 9_amct

## Directory structure and description

Ascend Model Compression Toolkit (AMCT) is a toolkit that compresses models through model compression techniques (such as fusion, quantization, tensor decomposition, etc.). After compression, the model volume becomes smaller and is deployed to Ascend AI After the processing device is installed, low-bit operations can be enabled to improve calculation efficiency and achieve the goal of performance improvement.

This catalog contains usage examples of the basic functions of AMCT tools. Since AMCT has developed tools based on different deep learning frameworks, this warehouse uses multiple folders to manage samples of different tools.
| directory  | description  |
|---|---|
| [amct_acl](./amct_acl/README_CN.md)  | Examples of amct_acl related functions |
| [amct_caffe](./amct_caffe/README_CN.md)  | Examples of amct_caffe related functions  |
| [amct_mindspore](./amct_mindspore/README_CN.md)  | Examples of amct_mindspore related functions  |
| [amct_onnx](./amct_onnx/README_CN.md)  | Examples of amct_onnx related functions  |
| [amct_pytorch](./amct_pytorch/README_CN.md)  | Examples of amct_pytorch related functions  |
| [amct_tensorflow](./amct_tensorflow/README_CN.md)  | Examples of amct_tensorflow related functions  |
| [amct_tensorflow_ascend](./amct_tensorflow_ascend/README_CN.md)  | Examples of amct_tensorflow_ascend related functions  |