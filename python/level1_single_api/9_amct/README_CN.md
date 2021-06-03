# 9_amct

## 介绍

昇腾模型压缩工具（Ascend Model Compression Toolkit，简称AMCT）是通过模型压缩技术（如融合，量化，张量分解等）将模型进行压缩的工具包，压缩后模型体积变小，部署到昇腾 AI 处理器件上后可使能低比特运算，提高计算效率，达到性能提升的目标。

本仓包含 AMCT 工具基本功能的使用样例，由于 AMCT 基于不同的深度学习框架分别开发了工具，本仓使用多个文件夹管理不同工具的样例。

**./**  
├── [amct_acl](./amct_acl/README_CN.md)**：amct_acl相关功能样例。**  
├── [amct_caffe](./amct_caffe/README_CN.md)**：amct_caffe相关功能样例。**  
├── [amct_mindspore](./amct_mindspore/README_CN.md)**：amct_mindspore 相关功能样例。**  
├── [amct_onnx](./amct_onnx/README_CN.md)**：amct_onnx 相关功能样例。**  
├── [amct_pytorch](./amct_pytorch/README_CN.md)**：amct_pytorch 相关功能样例。**  
├── [amct_tensorflow](./amct_tensorflow/README_CN.md)**：amct_tensorflow相关功能样例。**  
└── [amct_tensorflow_ascend](./amct_tensorflow_ascend/README_CN.md)**：amct_tensorflow_ascend相关功能样例。**
