# JPEG图片解码<a name="ZH-CN_TOPIC_0000001073131184"></a>

### 本样例为大家学习昇腾软件栈提供参考，非商业目的！

## 功能描述<a name="section09679311389"></a>

DVPP中的JPEGD功能模块，实现.jpg、.jpeg、.JPG、.JPEG图片的解码。该样例中：

输入图像：宽1920像素、高1080像素，名称为“dvpp_jpegd_decode_1920x1080.jpg”的JPEG图片。

输出图像：宽1920像素、高1080像素，名称为“chn0_1_outfile.yuv”的YUV420SP NV12格式图片。

## 原理介绍<a name="section19985135703818"></a>

样例中的关键接口调用流程如下：

![img](https://support.huaweicloud.com/aclcppdevg-cann51RC1alpha2/figure/zh-cn_image_0000001213553428.png)

## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── data
│   ├── dvpp_jpegd_decode_1920x1080.jpg     // 测试图片，需按照下文中的指导，获取测试图片后，放到data目录下

├── src
│   ├── main.cpp                            // 主函数实现文件
│   ├── common.h                            // 公共函数定义文件
│   ├── tinyjpegd.cpp                       // 解码功能的实现文件

├── output                                  // 按照下文中的指导，编译运行后，应用的可执行文件，解码后的图片存放在该目录下

├── CMakeLists.txt                          // 编译脚本

```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：Ubuntu 18.04 x86\_64、Ubuntu 18.04 aarch64、EulerOS x86、EulerOS aarch64
-   编译器：g++ 或 aarch64-linux-gnu-g++
-   芯片：Ascend 310P
-   已在环境上部署昇腾AI软件栈，并配置对应的的环境变量，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。

     以下步骤中，开发环境指编译开发代码的环境，运行环境指运行算子、推理或训练等程序的环境，运行环境上必须带昇腾AI处理器。开发环境和运行环境可以合设在同一台服务器上，也可以分设在不同的服务器上，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若两者服务器上的操作系统架构不同，则需要在开发环境中执行交叉编译。


## 准备测试数据<a name="section13133171616100"></a>

请从[https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data](https://github.com/Ascend/tools/tree/master/dvpp_sample_input_data)获取该样例的输入图片dvpp_jpegd_decode_1920x1080.jpg，并放至"tinyjpegd/data"目录下。

## 编译运行<a name="section13133171616172"></a>

1. **编译代码** 

    1. 以运行用户登录开发环境。

    2. 请先进入tinyjpegd样例目录。

    3. 设置环境变量，配置程序编译依赖的头文件与库文件路径。

          编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
   
          -   当运行环境操作系统架构是x86时，配置示例如下所示：
        
              ```
              export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
              export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
              ```
   
          -   当运行环境操作系统架构时AArch64时，配置示例如下所示：
        
              ```
              export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
              export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
              ```
   
    4. 切换到tinyjpegd目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build“。

        ```
        mkdir -p build
        ```

   
    5. 切换到“build“目录，执行以下命令生成编译文件。

        “..“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        将DCMAKE\_SKIP\_RPATH设置为TRUE，代表不会将rpath信息（即NPU_HOST_LIB配置的路径）添加到编译生成的可执行文件中去，可执行文件运行时会自动搜索实际设置的LD_LIBRARY_PATH中的动态链接库。

         -   当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

             ```
             cd build
             cmake .. -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
             ```
   
         -   当开发环境与运行环境操作系统架构不同时，执行以下命令进行交叉编译。

             例如，当开发环境为X86架构，运行环境为AArch64架构时，执行以下命令进行交叉编译。
          
             ```
             cd build
             cmake .. -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
             ```
          
          您可以登录对应的环境，执行“uname -a”命令查询其操作系统的架构。

    6. 执行以下命令，生成的可执行文件tiny_jpegd在“tinyjpegd/output“目录下。
   
       ```
       make
       ```
   
2. **运行应用** 

    1. 以运行用户将开发环境的tinyjpegd目录及目录下的文件上传到运行环境，例如“$HOME/tinyjpegd”。

    2. 以运行用户登录运行环境。

    3. 切换到可执行文件tiny_jpegd所在的目录，例如“$HOME/tinyjpegd/output”，给该目录下的tiny_jpegd文件加执行权限。

       ```
       chmod +x tiny_jpegd
       ```

     4. 切换到可执行文件tiny_jpegd所在的目录，例如“$HOME/tinyjpegd/output”，运行可执行文件。

        ```
        ./tiny_jpegd
        ```

         执行成功后，可在“$HOME/tinyjpegd/output”下查看解码后的YUV图片。

         用户可以下载第三方的YUV图像查看工具验证解码后的图片。
