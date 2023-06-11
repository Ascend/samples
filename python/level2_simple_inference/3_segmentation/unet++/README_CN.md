# 语义分割样例

## 介绍

本样例为基于Atlas 200I DK A2的语义分割样例。在该样例中，我们将一起学习如何把模型部署至DK中，并完成一个基于色块的语义分割样例。

## 环境

请在`https://www.hiascend.com/hardware/developer-kit-a2`页面完成Atlas 200I DK A2的购买。

完成购买后，请参考文档`https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC1/qs/qs_0003.html`，从一键制卡开始，完成基础镜像的制作。

## 运行

我们首先下载模型：`https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/ToTest/23.0.RC2/seg_sample.zip`。将下载好的.om文件放置在本项目的`model`文件夹中。

随后进入目录：`cd ${path_to_unet++}/unet++/infer/sdk`，打开一个命令行，执行命令`bash build.sh`。

等待样例执行完毕，我们查看`sdk`文件夹，发现该文件夹中新增了一个`infer_result`的文件夹。里面存放这我们刚刚推理结束的色块语义分割图像结果。
