## 目录

  - [样例介绍](#样例介绍)
  - [环境安装](#环境安装)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

  功能：演示mmdeploy如何使用部署工具将pth模型转换为om模型，并在NPU上加载om模型运行推理，此处分别提供mmdeploy图片分类库和图片检测库的样例运行。  
  图片分类：  
  &nbsp;&nbsp;    - 样例输入：图片  
  &nbsp;&nbsp;    - 样例输出：类别标签，置信度。  
  图片检测：  
  &nbsp;&nbsp;    - 样例输入：图片  
  &nbsp;&nbsp;    - 样例输出：图片，框选目标。

## 环境变量配置

  - 执行CANN环境变量set_env.sh，${HOME}为 "Ascend-cann-toolkit"安装位置
      ```
      source ${HOME}/Ascend/ascend-toolkit/set_env.sh  
      ```

  - mmdeploy 依赖及版本查询
      - gcc 7+        （gcc --version)
      - cmake >= 3.14  (cmake --version)  
      - opencv 3+      (opencv_version)
      ```      
      sudo apt-get install libopencv-dev 
      ```  
      - bz2及所需依赖
      ```   
      sudo apt-get install libbz2-dev 
      ```
      - python >= 3.7
      - 安装pytorch,mim,click  （pip3安装速度慢可使用源镜像）
      ```
      pip3 install --upgrade pip  
      pip3 install torch==1.8.1 torchvision==0.9.1 --extra-index-url https://download.pytorch.org/whl/cpu  
      pip3 install click==7.1.2 
      ```

  - mmdeploy 安装
      - 安装openmim,mmcv
      ```
      pip3 install openmim
      mim install mmcv-full 
      ```
      - 下载[mmdeploy](https://github.com/open-mmlab/mmdeploy)源码
      ```
      git clone --recursive https://github.com/open-mmlab/mmdeploy.git
      ```
      - 安装MMDeploy的模型转换器
      ```
      cd mmdeploy
      pip3 install -v -e .
      
      ```
      - 编译安装mmdeploy的SDK
      ```
      mkdir -p build && cd build
      cmake .. -DMMDEPLOY_BUILD_SDK=ON -DMMDEPLOY_BUILD_SDK_PYTHON_API=ON -DMMDEPLOY_TARGET_BACKENDS=acl
      make -j$(nproc)
      make install
      cd ../
      export PYTHONPATH=$(pwd)/build/lib:$PYTHONPATH
      ```
      - 检查环境信息可使用以下命令
      ```
      python3 tools/check_env.py
      ```

## 样例运行
  mmdeploy包含丰富的深度学习模型部署工具，包括MMCls,MMDet,MMSeg,MMEdit,MMOCR,MMDet3D,MMPose,MMRotate等
  分别以MMCls,MMDet为例，提供模型转换和推理。

#### 分类模型
  - 安装MMCls算法库，并下载MMCls库中的resnet18模型到目标文件夹samples
    ```
    pip3 install mmcls
    mim download mmcls --config resnet18_8xb32_in1k --dest samples
    ```    
  - 执行mmdepoly转换命令，得到 end2end.om模型及mmdeploy model, --work-dir 为模型转换后输出目录
    ```
    python3 tools/deploy.py \  
        configs/mmcls/classification_ascend_static-224x224.py \  
        samples/resnet18_8xb32_in1k.py \
        samples/resnet18_8xb32_in1k_20210831-fbbb1da6.pth \  
        tests/data/tiger.jpeg \  
        --work-dir mmdeploy_models/mmcls/resnet18/cann \  
        --device cpu \  
        --dump-info
    ```
  - om模型推理
    移动本样例中的 image_classification.py 到 ./mmdeploy/ 目录下，传入om模型目录，传入输入图片(大小不限)，获取输出图片
    ```
    python3 image_classification.py
    ```
  - 输出结果  
    ```
    labels  Confidence
    292     0.92626953125
    282     0.07257080078125
    290     0.0008058547973632812
    281     0.00024580955505371094
    340     5.65648078918457e-05
    ```
                  
#### 检测模型
  - 安装MMDet算法库，并下载MMDete库中的faster-rcnn模型到目标文件夹samples。
    ```
    pip3 install mmdet
    mim download mmdet --config faster_rcnn_r50_fpn_1x_coco --dest samples
    ```    
  - 执行mmdepoly转换命令，得到 end2end.om模型及mmdeploy model，其中 --work-dir 为模型转换后输出目录
    ```
    python3 tools/deploy.py \
        configs/mmdet/detection/detection_ascend_static-800x1344.py \
        samples/faster_rcnn_r50_fpn_1x_coco.py \
        samples/faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth \
        tests/data/tiger.jpeg \
        --work-dir mmdeploy_models/mmdet/faster-rcnn/cann \
        --device cpu \
        --dump-info
    ```
  - om模型推理：  
    移动本样例中的 object_detection.py 到 mmdeploy 仓目录下，传入om模型目录，传入输入图片(大小不限)，获取输出图片
    ```
    python3 object_detection.py
    ```
  - 输出结果  
                ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/contributeSamples/paddle/output_detection.png)

## 其他资源

以下资源提供了对mmdeploy项目和昇腾NPU的更深入理解：

**mmdeploy**
- [GitHub: mmdeploy](https://github.com/open-mmlab/mmdeploy)

**Documentation**

- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/03/8 | 更新ReadMe.md|
  
## 已知issue
  - 当前mmdeploy版本运行faster-rcnn推理时，存在如下问题，可通过以下方法规避解决。  
    https://github.com/Ascend/samples/issues/I6J9O3?from=project-issue
