# readme<a name="ZH-CN_TOPIC_0000001072529927"></a>

### 本样例为大家学习昇腾软件栈提供参考，非商业目的！
### 本样例适配5.0.3及以上版本，支持产品为710设备。


## 功能描述<a name="section09679311389"></a>

DVPP中的JPEGE功能模块，实现将YUV格式图片编码成.jpg图片。

## 原理介绍<a name="section19985135703818"></a>

样例中的关键接口调用流程如下：

![输入图片说明](https://images.gitee.com/uploads/images/2020/1225/093219_44fe5420_8492653.png "zh-cn_image_0000001072787903.png")

## 目录结构<a name="section86232112399"></a>

```
├──————CMakeLists.txt        // 编译脚本
├──————common                // 示例代码文件所在的目录
├──————smoke_jpege           // 示例代码文件所在的目录
```

## 环境要求<a name="section10528164623911"></a>

-   操作系统及架构：Ubuntu 18.04 x86\_64、Ubuntu 18.04 aarch64、EulerOS aarch64
-   编译器：

    EP标准形态编译器：g++

    EP开放形态编译器：hcc编译器aarch64-target-linux-gnu-g++\(在toolkit包中\)

-   芯片：Ascend710
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。

## 准备测试数据<a name="section13765133092318"></a>

请从[https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data](https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data)获取该样例的输入图片、视频数据。

## 编译运行<a name="section3789175815018"></a>

1. 以运行用户登录开发环境，编译代码。

   1. 设置环境变量，编译脚本CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

      如下为示例，$HOME/Ascend表示runtime标准形态安装包的安装路径，请根据实际情况替换。

      ```
      export DDK_PATH=$HOME/Ascend
      export NPU_HOST_LIB=$HOME/Ascend/runtime/lib64/stub/
      ```

    2. 切换到jpege\_sample所在目录，依次执行如下命令执行编译。

       ```
       mkdir build
       cd build
       cmake .. -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
       make
       ```

       在“jpege\_sample/build/src“目录下会生成可执行程序jpege\_demo。



2. 以运行用户将开发环境的“jpege\_sample/build/src“目录下的可执行程序jpege\_demo以及[准备测试数据](#section13765133092318)中测试数据上传到运行环境（Host）的同一目录下，例如“$HOME/dvpp/jpege\_sample“。

3. 运行应用。

   1. 切换到可执行文件jpege\_demo所在的目录，例如“$HOME/dvpp/jpege\_sample“，给该目录下的jpege\_demo文件加执行权限。

      ```
      chmod +x  jpege_demo
      ```

   2. 设置环境变量。“$HOME/Ascend“表示runtime标准形态安装包的安装路径，请根据实际情况替换。

      ```
      export LD_LIBRARY_PATH=$HOME/Ascend/runtime/lib64
      ```

   3. <a name="li163081446765"></a>运行应用。

      - 示例描述：使用JPEGE编码器将dvpp\_venc\_128x128\_nv12.yuv编码为jpg图片。

      - 输入图像：宽128像素、高128像素、名称为“dvpp\_venc\_128x128\_nv12.yuv”的YUV420sp数据。

      - 输出图像：宽128像素、高128像素的jpg图片，名称为snap_chnl0_no0.jpg ，表示通道0的第0张图像。

      - 运行应用的命令示例如下：

        ```
        ./jpege_demo --in_image_file ./dvpp_venc_128x128_nv12.yuv --img_width 128 --img_height 128 --in_format 1 --chn_num 1
        ```

      - 运行可执行文件的通用参数说明如下所示：

        - img\_width：输入图片的宽度，范围\[32, 8192\]。
        - img\_height：输入图片的高度，范围\[32, 8192\]。
        - chn\_width：创建通道的宽度, 范围\[32, 8192\]。如果用户没有传入通道宽高，默认使用图像宽高作为通道宽高。
        - chn\_height：创建通道的高度, 范围\[32, 8192\]。如果用户没有传入通道宽高，默认使用图像宽高作为通道宽高。
        - in\_format：YUV数据格式。
          - 1：YUV420SP
          - 2：YVU420SP
          - 7：YUYV422PACKED
          - 8：UYVY422PACKED
          - 9：YVYU422PACKED
          - 10：VYUY422PACKED
        - chn\_num：创建编码通道的数目，最大不得超过128路。
        - in\_image\_file：输入图像文件的路径，包含文件名。
        - save：是否保存输出码流。
          - 默认1，0：不保留（主要用于性能测试）
          - 非0：保留
        - chn\_start：编码起始通道号，范围\[0, 127\]，不指定则从0开始。
        - performance：性能测试标识。
          - 默认0：功能测试
          - 非0：性能测试
        - one\_thread：通知用户取走输出码流。
          - 默认0：单线程多通道
          - 非0：单线程单通道
        - level：编码质量，默认值100，取值范围\[1, 100\]
        - overtime：编码帧数与发送帧数比较的次数，小于设定值则继续等待编码完成，大于设定值则认为编码超时，建议值30。
        - per\_count：性能统计循环次数，默认值300，取值范围\[1, ∞\)。
