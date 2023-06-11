中文|[English](README.md)

<div align="center">
<p align="center">
  <img src="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/samples_pic/CANN_samples.png" align="middle" width = "800" />
</p>
</div>


## 简介

昇腾样例仓提供了一系列算子、推理与训练领域的丰富代码样例给开发者进行参考学习，帮助开发者快速进行落地应用。在开发者朋友们开发自己的样例时，也可以就样例仓的相关案例进行参考。

部分样例效果示例如下（点击标题可快速跳转）：

|                                                  [**卡通图像生成**](https://github.com/Ascend/samples/tree/master/python/contrib/cartoonGAN_picture)                                                  |                                                [**AI 风景画**](https://github.com/Ascend/samples/tree/master/cplusplus/contrib/AI_painting)                                                |                                                  [**图像消消消**](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/6_other/imageinpainting_hifill)                                                  |                                            [**黑白图像上色**](https://github.com/Ascend/samples/tree/master/cplusplus/level2_simple_inference/6_other/colorization)                                            |
| :--------------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------: |
| <img src='https://r.huaweistatic.com/s/ascendstatic/lst/public/img/reference-design/demo/cartoon.png' height="126px" width="180px"> | <img src='https://r.huaweistatic.com/s/ascendstatic/lst/mindx-sdk/demo/landscape/landscape.png' height="126px" width="180px"> |  <img src='https://r.huaweistatic.com/s/ascendstatic/lst/mindx-sdk/demo/remove/show.png' height="126px" width="180px">  | <img src='https://r.huaweistatic.com/s/ascendstatic/lst/public/img/reference-design/demo/h-w-pic.png' height="126px" width="180px"> |



## 版本说明

**请在[硬件平台页面](https://www.hiascend.com/hardware/firmware-drivers?tag=community)选择您使用的产品后，通过下拉框选择支持的CANN版本并查看配套关系。**

- **当前分支样例适配的CANN版本 [>=6.0.RC1.alpha005](https://www.hiascend.com/software/cann/community)**    
   
    | 时间 | 更新事项 |
    |----|------|
    | 2023/05/23   | 新增样例：[sampleYOLOV7NMSONNX](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7NMSONNX)
    | 2023/05/17   | 新增样例：[sampleCrowdCounting](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCrowdCounting)
    | 2023/05/16   | 样例新增功能点：[sampleYOLOV7MultiInput](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7MultiInput)
    | 2023/05/16   | 新增样例：[sampleCarColor](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCarColor)
    | 2023/05/11   | 新增样例：[sampleResnetRtsp](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetRtsp)
    | 2023/04/10   | 新增初级成长路径：[growthpath](./growthpath) |
    | 2023/03/29   | 新增标签：[v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/) |
    | 2023/03/10   | 新增样例：[ACLHelloWorld](https://github.com/Ascend/samples/tree/master/inference/ACLHelloWorld) |
    | 2023/03/09   | 新增样例：[sampleYOLOV7](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7)、[sampleMMDeploy](https://github.com/Ascend/samples/tree/master/inference/contributeSamples/contrib/samplesMMDeploy)|
    | 2023/02/17   | 新增样例：[sampleResnetQuickStart](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)、[sampleResnetAIPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)、[sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)  |
    | 2023/02/10   | 新增目录分支：[operator](https://github.com/Ascend/samples/tree/master/operator)、[inference](https://github.com/Ascend/samples/tree/master/inference)、[training](https://github.com/Ascend/samples/tree/master/training) 分别存放算子、推理、训练相关样例及指导。  |
  
- **历史版本请参考[历史版本信息](docs/CHANGELOG.md)**  


## 环境要求

-   操作系统及架构：CentOS、Ubuntu、EulerOS
-   编译器：g++ 或 aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
- 固件与驱动
    
    在[硬件产品文档](https://www.hiascend.com/document?tag=hardware)中选择对应的硬件产品文档，参考**NPU驱动固件安装指南**进行硬件安装。
- CANN软件安装

    参考[CANN 开发者文档](https://www.hiascend.com/document?tag=hardware)中**CANN软件安装**章节进行CANN包的安装。


## 目录结构与说明
- docs
    
    该目录下主要存放所有说明文档。

- inference

    | 子目录名称                                                   | 功能描述                                             |
    | ------------------------------------------------------------ | ---------------------------------------------------- |
    | [msame](https://github.com/Ascend/samples/tree/master/inference/msame) | 一种快速利用AscendCL的工具，无需开发自己的应用程序 |
    | [memoryManagement](https://github.com/Ascend/samples/tree/master/inference/memoryManagement) | 内存管理样例 |
    | [mediaProcess](https://github.com/Ascend/samples/tree/master/inference/mediaProcess) | 媒体数据处理样例 |
    | [modelInference](https://github.com/Ascend/samples/tree/master/inference/modelInference) | 模型推理样例 |
    | [moreModelFeatures](https://github.com/Ascend/samples/tree/master/inference/moreModelFeatures) | 使用更多特性的模型推理样例 |
    | [contributeSamples](https://github.com/Ascend/samples/tree/master/inference/contributeSamples) | 贡献样例 |
    | [acllite](https://github.com/Ascend/samples/tree/master/inference/acllite) | acllite统一封装接口 |
    | [common](https://github.com/Ascend/samples/tree/master/inference/common) | 存放第三方库presenteragent&presenterserver |

- operator

    | 子目录名称                                                   |       功能描述       |
    | :----------------------------------------------------------- | :------------------: |
    | [op_implementation](https://github.com/Ascend/samples/tree/master/operator/op_using) |   算子开发指导说明 |
    | [op_using](https://github.com/Ascend/samples/tree/master/operator/op_using) | 算子编译推理应用样例 |

- training

    该目录下主要存放模型训练相关的操作说明指引。


## 教程

- [初级成长路径](./growthpath/junior-level)
- [中级成长路径](docs/INSTALL_cn.md)
- [高级成长路径](docs/INSTALL_cn.md)

## FAQ

- [FAQ/常见问题汇总](docs/FAQ)

## 产业实践范例

<details>
<summary><b> ISV经典样例(点击展开)</b></summary>
</details>

<details>
<summary><b> 高校经典样例(点击展开)</b></summary>
</details>


## 开源社区

- 活动/直播课程

- 微信公众号推文链接/热点跟随文章/B站视频更新微信公众号推文链接/热点跟随文章/B站视频更新

- 社区贡献
    
    欢迎参与贡献。更多详情，请参阅我们的[贡献者Wiki](./CONTRIBUTING_CN.md)。

- 加入社区

    参考社区网站[昇腾文档](https://www.hiascend.com/zh/document)获取相关文档。

    昇腾社区鼓励开发者多交流，共学习。开发者可以通过以下渠道进行交流和学习。
    
    昇腾社区网站：hiascend.com
    
    昇腾论坛：https://www.hiascend.com/forum/
    
    昇腾官方qq群：965804873


## 许可证

[Apache License 2.0](LICENSE)


