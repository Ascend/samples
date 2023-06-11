## 目录

  - [样例介绍](#样例介绍)
  - [第三方依赖安装](#第三方依赖安装)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

  演示如何在昇腾310上编译OpenCV库，并使用opencv_zoo的mobilenetv1模型,通过npu后端进行推理，获取图片分类结果。  
  图片分类：  
  &nbsp;&nbsp;    - 样例输入：图片  
  &nbsp;&nbsp;    - 样例输出：类别标签，分类结果。  

## 第三方依赖安装

  - Ascend-cann-toolkit版本 [5.1.RC2.alpha008](https://www.hiascend.com/software/cann/community-history)

  - 执行CANN环境变量set_env.sh，${HOME}为 "Ascend-cann-toolkit"安装位置
      ```
      source ${HOME}/Ascend/ascend-toolkit/set_env.sh  
      ```
  - OpenCV 依赖
      - gcc 7.3+        （gcc --version)
      - cmake >= 3.5  (cmake --version)  
      - make
      - python >= 3.7
      ```
      export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
      export PATH=/usr/local/python3.7.5/bin:$PATH
      ```
  - 安装编译 opencv 源码
      ```
      git clone https://github.com/opencv/opencv.git
      cd opencv
      mkdir build && cd build
      cmake -D WITH_CANN=ON \
            -D CMAKE_INSTALL_PREFIX=install \
            -D BUILD_opencv_gapi=OFF \
            -D OPENCV_DOWNLOAD_MIRROR_ID=gitcode \
            -D BUILD_opencv_python2=OFF \
            -D BUILD_opencv_python3=ON \
            -D PYTHON3_EXECUTABLE=/usr/local/python3.7.5/bin/python3.7m \
            -D PYTHON3_LIBRARY=/usr/local/python3.7.5/lib/libpython3.7m.so \
            -D PYTHON3_INCLUDE_DIR=/usr/local/python3.7.5/include/python3.7m \
            ..
      # ensure you see 'CANN: YES' in the end of the log
      # Note: you could append "-j 8" in the following command for multi-job speedup.
      # More jobs are used, more memory is needed.
      cmake --build . --target install -j8
      cd ..
      ```
  - 进入opencv目录，添加 CMAKE_PREFIX_PATH 环境变量
      ```
      export CMAKE_PREFIX_PATH=$(pwd)/build/install
      ```
## 样例运行
  [opencv_zoo](https://github.com/opencv/opencv)提供给了丰富的推理模型可在昇腾npu上进行推理，包括物体分类，目标检测，人脸识别，姿态检测等等，此处我们选用提供的[mobilenetv1_2022.onnx](https://github.com/opencv/opencv_zoo/raw/master/models/image_classification_mobilenet/image_classification_mobilenetv1_2022apr.onnx)演示如何在npu上进行图片分类

#### 案例下载 
  - 下载samples仓，进入本案例目录
    ```
    git clone https://github.com/Ascend/samples
    cd samples/inference/contributeSamples/contrib/samplesOpenCV/
    ```

#### 获取模型和数据
  - 下载模型到model目录
    ```
    cd model
    wget https://github.com/opencv/opencv_zoo/raw/master/models/image_classification_mobilenet/image_classification_mobilenetv1_2022apr.onnx
    ```
  - 下载测试图片到data目录 
    ```
    cd ../data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg
    ```

#### 使用c++编译执行
    进入scripts目录，运行脚本编译mobilenetv1.cpp文件，加载数据、模型，输出得分最高的分值和lable下标。
  - 编译
    ```
    cd scripts
    bash sample_build.sh
    ```    
  - 运行
    ```
    bash sample_run.sh
    ```
  - 输出结果  
    ```
    cls = 163, score = 0.4744
    ```                  

## 其他资源

以下资源提供了对OpenCV项目和昇腾NPU的更深入理解：

**opencv库**
- [GitHub: opencv](https://github.com/opencv/opencv)

**opencv_zoo模型库**
- [GitHub: opencv_zoo](https://github.com/opencv/opencv_zoo)

**Documentation**

- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/03/20 | 更新ReadMe.md|
  
## 已知issue
   无
