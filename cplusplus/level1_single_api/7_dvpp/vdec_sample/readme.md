# readme<a name="ZH-CN_TOPIC_0000001072850082"></a>

### 本样例为大家学习昇腾软件栈提供参考，非商业目的！
### 本样例适配5.0.3及以上版本，支持产品为710设备。


## 功能描述<a name="section09679311389"></a>

DVPP中的VDEC功能模块，实现H264码流格式的视频解码。

## 原理介绍<a name="section19985135703818"></a>

样例中的关键接口调用流程如下：

![输入图片说明](https://images.gitee.com/uploads/images/2020/1225/093344_3b09ceea_8492653.png "zh-cn_image_0000001073316690.png")

## 目录结构<a name="section86232112399"></a>

```
├──————CMakeLists.txt        // 编译脚本
├──————VdecDemo.cpp          // 示例代码文件
├──————Vdec.cpp              // 示例代码文件
├──————Vdec.h                // 头文件
```

## 环境要求<a name="section10528164623911"></a>

- 操作系统及架构：Ubuntu 18.04 x86\_64、Ubuntu 18.04 aarch64、EulerOS aarch64

-   编译器：

    EP标准形态编译器：g++

    EP开放形态编译器：hcc编译器aarch64-target-linux-gnu-g++\(在toolkit包中\)

- 芯片：Ascend710

- 已完成昇腾AI软件栈在开发环境、运行环境上的部署。


## 准备测试数据<a name="section13765133092318"></a>

请从[https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data](https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data)获取该样例的输入图片、视频数据。

## 编译运行<a name="section3789175815018"></a>

1. 以运行用户登录开发环境，编译代码。

   1. 设置环境变量，编译脚本CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

      如下为示例，"$HOME/Ascend"表示runtime标准形态安装包的安装路径，请根据实际情况替换。

      ```
      export DDK_PATH=$HOME/Ascend
      export NPU_HOST_LIB=$HOME/Ascend/runtime/lib64/stub/
      ```

   2. 依次执行如下命令执行编译。

      ```
      mkdir build
      cd build
      cmake .. -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
      make
      ```

      在“vdec\_sample/build“目录下会生成可执行程序vdec\_demo。

2. 以运行用户将开发环境的“vdec\_sample/build“目录下的可执行程序vdec\_demo以及[准备测试数据](#section13765133092318)中测试数据拷贝到运行环境（Host）的同一目录下，例如“$HOME/dvpp/vdec\_sample“。

3. 运行应用。

   1. 切换到可执行文件vdec\_demo所在的目录，例如“$HOME/dvpp/vdec\_sample“，给该目录下的vdec\_demo文件加执行权限。

      ```
      chmod +x vdec_demo
      ```

   2. 设置环境变量。“$HOME/Ascend“表示runtime标准形态安装包的安装路径，请根据实际情况替换。

      ```
      export LD_LIBRARY_PATH=$HOME/Ascend/runtime/lib64
      ```

   3. <a name="li163081446762"></a>运行应用。

      运行可执行文件的命令示例及说明请参见[单路H264码流解码](#section77661251154615)\~[单路H264码流解码+缩放](#section3560932124812)。

      使用demo进行性能测试时不能保存解码结果，需要配置--write\_file 0。使用demo进行多路性能测试时，为了多路同时并发，需要设置启动延迟时间--delay\_time 20，以保证码流发送线程在同一时刻开始发送码流。

      运行可执行文件的参数说明如下所示：

      - img\_height：输入视频的高度。默认值为2160。取值范围\[128, 4096\]。
        - in\_image\_file：输入图像文件路径，包含文件名。默认值为“infile”。
        - in\_format：输入码流的格式。
          - 默认值为0：0：h264格式的码流。
          - 1：h265格式的码流。
        - in\_bitwidth：码流的位宽。默认值为8。
          - 8：8bit码流。
          - 10：10bit码流。
      - chn\_num：创建解码通道的数目，默认值为1。取值范围\[1, 96\]。
        - out\_width：输出图像宽度。支持缩放，0表示不缩放原图输出，默认值为0。取值范围\[10, 4096\]。
        - out\_height：输出图像高度。支持缩放，0表示不缩放原图输出，默认值为0。取值范围\[6, 4096\]。
        - out\_image\_file：输出图像文件的路径，包含文件名。默认值为“outfile”。
        - out\_format：输出图像的格式。默认值为0。
          - 0：YUV420SP
          - 1：YVU420SP
          - 2：RGB888
        - 3：BGR888
        - width\_stride：输出内存宽Stride，要求16对齐，其中RGB888/BGR888最小width\_stride = AlignUp\(out\_width, 16\) \* 3。默认值为4096。
        - height\_stride：输出内存高Stride，YUV420SP/YVU420SP格式要求2对齐，默认值为4096。
      - ref\_frame\_num：参考帧个数。
        - dis\_frame\_num：显示帧个数。
        - write\_file：是否保存输出解码结果：
          - 1：保存
          - 0：不保存
        - output\_order：解码图像输出顺序。默认值为0。
          - 0：显示序
          - 1：解码序
        - send\_times：输入码流循环发送次数。默认值为1，设置0表示一直循环。
        - send\_interval：码流每帧发送时间间隔，单位us。默认值为0。
        - delay\_time：码流发送线程启动发送码流的时延，单位s，默认值为1。
        - alloc\_num：每路解码的输出内存预申请个数，默认值为20。
        - start\_chn：多路解码起始通道号，默认值为0。
        - render：抽帧间隔，0表示每帧都进行后处理，默认值为0。


## 单路H264码流解码<a name="section77661251154615"></a>

```
./vdec_demo --in_image_file ./dvpp_vdec_h264_1frame_bp_51_1920x1080.h264 --img_width 1920 --img_height 1080 --in_format 0 --in_bitwidth 8 --chn_num 1 --out_width 1920 --out_height 1080 --width_stride 1920 --height_stride 1080 --out_format 0 --out_image_file ./yuv_1920x1080_1frames_h264 --ref_frame_num 5 --dis_frame_num 3 --write_file 1
```

-   示例描述：本命令用于调用VDEC解码h264格式的码流，得到YUV420SP格式图像。
-   输入视频：宽1920像素、高1080像素、名称为“dvpp\_vdec\_h264\_1frame\_bp\_51\_1920x1080.h264”的码流。
-   输出图像：宽1920像素、高1080像素、格式为YUV420SP的图像。

## 16路H264码流解码+输出RGB888格式图片<a name="section1648922318485"></a>

```
./vdec_demo --in_image_file ./dvpp_vdec_h264_1frame_bp_51_1920x1080.h264 --img_width 1920 --img_height 1080 --in_format 0 --in_bitwidth 8 --chn_num 16 --out_width 1920 --out_height 1080 --width_stride 5760 --height_stride 1080 --out_format 2 --out_image_file ./yuv_1920x1080_1frames_h264 --ref_frame_num 5 --dis_frame_num 3 --write_file 1
```

-   示例描述：本命令用于16路调用VDEC解码h264格式的码流，得到RGB888格式图像。
-   输入视频：宽1920像素、高1080像素、名称为“dvpp\_vdec\_h264\_1frame\_bp\_51\_1920x1080.h264”的码流。
-   输出图像：宽1920像素、高1080像素、格式为RGB888的图像。

## 单路H264码流解码+缩放<a name="section3560932124812"></a>

```
./vdec_demo --in_image_file ./dvpp_vdec_h264_1frame_bp_51_1920x1080.h264 --img_width 1920 --img_height 1080 --in_format 0 --in_bitwidth 8 --chn_num 1 --out_width 128 --out_height 128 --width_stride 128 --height_stride 128 --out_format 0 --out_image_file ./yuv_1920x1080_1frames_h264 --ref_frame_num 5 --dis_frame_num 3 --write_file 1
```

-   示例描述：本命令用于调用VDEC解码h264格式的码流，并进行缩放得到YUV420SP格式图像。
-   输入视频：宽1920像素、高1080像素、名称为“dvpp\_vdec\_h264\_1frame\_bp\_51\_1920x1080.h264”的码流。
-   输出图像：宽128像素、高128像素、格式为YUV420SP的图像。

