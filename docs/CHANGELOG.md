## 历史版本信息

- **查询CANN软件包的版本信息**

    参考[查询软件包版本信息](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha006/softwareinstall/instg/atlasdeploy_03_0079.html)

- **历史版本操作说明**      
  **tag**：对某一时间点的代码仓打标签，在发布某个软件版本（比如 v0.1.0 等等）时使用tag标签，给仓库中的项目添加tag。可以理解为某一时刻的不会变化的分支。   
  **Realease**：基于tag，为tag添加更丰富的信息，一般是编译好的文件。     
  1. 用户可以在仓库的分支切换框中选择对应的标签（tag）从而查看对应版本的代码及readme。    
  2. 用户可以下载realease提供的编译好的文件（Source code）进行代码使用。    
  3. 如果需要在命令行访问tag代码，可以按照如下方式操作。
     ```
     # 命令行下载master代码
     git clone https://github.com/Ascend/samples.git   
     # 切换到历史tag，以v0.1.0举例
     git checkout v0.1.0
     ```

- **历史版本请参考下表使用对应发行版：**      

    | CANN版本 | tag | Release |
    |---|---|---|
    | [6.3.RC1.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.0.alpha006](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.0.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.0.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.0.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.0.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.1.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.RC1.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/)  | [下载Release 0.9.0发行版](https://gitee.com/ascend/samples/releases/v0.9.0)   |
    | [6.0.RC1.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.8.0](https://gitee.com/ascend/samples/tree/v0.8.0/)  | [下载Release 0.8.0发行版](https://gitee.com/ascend/samples/releases/v0.8.0)   |
    | [6.0.RC1.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.8.0](https://gitee.com/ascend/samples/tree/v0.8.0/)  | [下载Release 0.8.0发行版](https://gitee.com/ascend/samples/releases/v0.8.0)   |
    | [6.0.RC1.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.8.0](https://gitee.com/ascend/samples/tree/v0.8.0/)  | [下载Release 0.8.0发行版](https://gitee.com/ascend/samples/releases/v0.8.0)   |
    | [5.1.RC2.alpha008](https://www.hiascend.com/software/cann/community)  | [v0.8.0](https://gitee.com/ascend/samples/tree/v0.8.0/)  | [下载Release 0.8.0发行版](https://gitee.com/ascend/samples/releases/v0.8.0)   |
    | [5.1.RC2.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.1.RC2.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.1.RC2.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.1.RC2.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.1.RC1.alpha006](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.1.RC1.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.7.0](https://gitee.com/ascend/samples/tree/v0.7.0/)  | [下载Release 0.7.0发行版](https://gitee.com/ascend/samples/releases/v0.7.0)   |
    | [5.0.4.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.0.4.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.0.4.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.0.5.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.1.RC1.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.1.RC1.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.1.RC1.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.6.0](https://gitee.com/ascend/samples/tree/v0.6.0/)  | [下载Release 0.6.0发行版](https://gitee.com/ascend/samples/releases/v0.6.0)  |
    | [5.0.2.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.5.0](https://gitee.com/ascend/samples/tree/v0.5.0/)  | [下载Release 0.5.0发行版](https://gitee.com/ascend/samples/releases/v0.5.0)  |
    | [5.0.3.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.5.0](https://gitee.com/ascend/samples/tree/v0.5.0/)  | [下载Release 0.5.0发行版](https://gitee.com/ascend/samples/releases/v0.5.0)  |
    | [5.0.3.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.5.0](https://gitee.com/ascend/samples/tree/v0.5.0/)  | [下载Release 0.5.0发行版](https://gitee.com/ascend/samples/releases/v0.5.0)  |
    | [5.0.3.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.5.0](https://gitee.com/ascend/samples/tree/v0.5.0/)  | [下载Release 0.5.0发行版](https://gitee.com/ascend/samples/releases/v0.5.0)  |
    | [5.0.3.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.5.0](https://gitee.com/ascend/samples/tree/v0.5.0/)  | [下载Release 0.5.0发行版](https://gitee.com/ascend/samples/releases/v0.5.0)  |
    | [5.0.2.alpha003](https://www.hiascend.com/software/cann/community)  | [v0.4.0](https://gitee.com/ascend/samples/tree/v0.4.0/)  | [下载Release 0.4.0发行版](https://gitee.com/ascend/samples/releases/v0.4.0)  |
    | [3.3.0.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.3.0](https://gitee.com/ascend/samples/tree/v0.3.0/)  | [下载Release 0.3.0发行版](https://gitee.com/ascend/samples/releases/v0.3.0)  |
    | [3.3.0.alpha005](https://www.hiascend.com/software/cann/community)  | [v0.3.0](https://gitee.com/ascend/samples/tree/v0.3.0/)  | [下载Release 0.3.0发行版](https://gitee.com/ascend/samples/releases/v0.3.0)  |
    | [3.3.0.alpha006](https://www.hiascend.com/software/cann/community)  | [v0.3.0](https://gitee.com/ascend/samples/tree/v0.3.0/)  | [下载Release 0.3.0发行版](https://gitee.com/ascend/samples/releases/v0.3.0)  |
    | [5.0.2.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.3.0](https://gitee.com/ascend/samples/tree/v0.3.0/)  | [下载Release 0.3.0发行版](https://gitee.com/ascend/samples/releases/v0.3.0)  |
    | [5.0.2.alpha002](https://www.hiascend.com/software/cann/community)  | [v0.3.0](https://gitee.com/ascend/samples/tree/v0.3.0/)  | [下载Release 0.3.0发行版](https://gitee.com/ascend/samples/releases/v0.3.0)  |
    | [3.2.0.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.2.0](https://gitee.com/ascend/samples/tree/v0.2.0/)  | [下载Release 0.2.0发行版](https://gitee.com/ascend/samples/releases/v0.2.0)  |
    | [3.1.0.alpha001](https://www.hiascend.com/software/cann/community)  | [v0.1.0](https://gitee.com/ascend/samples/tree/v0.1.0/)  | [下载Release 0.1.0发行版](https://gitee.com/ascend/samples/releases/v0.1.0)  |
    
- **更新事项**
    | 时间 | 更新事项 |
    |----|------|
    | 2023/05/23   | 新增样例：[sampleYOLOV7NMSONNX](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7NMSONNX)
    | 2023/05/17   | 新增样例：[sampleCrowdCounting](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCrowdCounting)
    | 2023/05/16   | 样例新增功能点：[sampleYOLOV7MultiInput](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7MultiInput)
    | 2023/05/16   | 新增样例：[sampleCarColor](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCarColor)
    | 2023/05/11   | 新增样例：[sampleResnetRtsp](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetRtsp) |
    | 2023/03/29   | 新增标签：[v0.9.0](https://gitee.com/ascend/samples/tree/v0.9.0/) |
    | 2023/03/10   | 新增样例：[ACLHelloWorld](https://github.com/Ascend/samples/tree/master/inference/ACLHelloWorld) |
    | 2023/03/09   | 新增样例：[sampleYOLOV7](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7)、[sampleMMDeploy](https://github.com/Ascend/samples/tree/master/inference/contributeSamples/contrib/samplesMMDeploy)|
    | 2023/02/17   | 新增样例：[sampleResnetQuickStart](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)、[sampleResnetAIPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)、[sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)  |
    | 2023/02/10   | 新增目录分支：[operator](https://github.com/Ascend/samples/tree/master/operator)、[inference](https://github.com/Ascend/samples/tree/master/inference)、[training](https://github.com/Ascend/samples/tree/master/training) 分别存放算子、推理、训练相关样例及指导。  |
