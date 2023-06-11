## 外部贡献推理样例说明

以下示例显示如何在Ascend-NPU上运行其他各平台的推理模型

<!--注：此处展示所支撑平台-->

| Title  |  Samples Name  | Description  |
|---|---|---|
| Classification and Detection for MMdeploy | [samplesMMDploy](https://github.com/Ascend/samples/tree/master/inference/contributeSamples/contrib/samplesMMDeploy)  | 将mmdeploy分类和检测模型运行在NPU上  |
| Classification for OpenCV-DNN | [samplesOpenCV](https://github.com/Ascend/samples/tree/master/inference/contributeSamples/contrib/samplesOpenCV)  | 演示如何在昇腾310上编译OpenCV库，并使用opencv_zoo的mobilenetv1模型,通过npu后端进行推理，获取图片分类结果  |
| Classification for Paddle-Lite | [samplesPaddle](https://github.com/Ascend/samples/tree/master/inference/contributeSamples/contrib/samplesPaddle)  | 演示如何在昇腾310上使用docker镜像配置环境，并通过编译好的paddlelite-demo包，在npu上使用paddle-lite做模型推理应用。 |