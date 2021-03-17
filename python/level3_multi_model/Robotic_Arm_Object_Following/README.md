**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.1.0及以上版本，支持产品为Atlas200DK。**

## 案例描述
使用Atlas200DK运行Yolov3模型，对双目深度相机给出的RGB数据流进行推理，实时检测目标在图像中的位置。并结合相机的深度数据流，控制机械臂的姿态，使得机械臂跟随目标移动。

## 物料清单
|**物料类型**|**详细描述**|
|---|---|
|推理平台|[Atlas200DK](https://support.huaweicloud.com/Atlas200DK202/)|
|双目深度相机|[Realsense D435](https://www.intelrealsense.com/depth-camera-d435/)|
|机械臂|[Dobot-Magician](https://cn.dobot.cc/dobot-magician/product-overview.html)|
|主控平台|Ubuntu18.04 LTS-x64-Huawei Matebook X Pro|
|路由器|Huawei 4G无线路由器B311|
|检测目标|[Ascend图标](https://images.gitee.com/uploads/images/2021/0310/112721_0db30e79_5537256.png)|

## 环境准备
### Ubuntu主控平台准备
- 参考[Atlas200DK说明文档]( https://support.huaweicloud.com/Atlas200DK202/ )部署开发环境
### Atlas200DK推理平台准备
- 参考[Atlas200DK说明文档]( https://support.huaweicloud.com/Atlas200DK202/ )部署运行环境
- 参考Ascend案例如[YOLOV3_coco_detection_picture](https://gitee.com/ascend/samples/tree/master/python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture#yolov3_coco_detection_picture%E6%A0%B7%E4%BE%8B )测试推理平台是否搭建成功
### Realsense双目相机准备
- 参考[Realsense SDK 2.0安装说明]( https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide )安装SDK
- 参考Realsense 使用示例如[opencv_viewer_example]( https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py )测试SDK是否安装成功
### Dobot-Magician机械臂准备
- 参考[pydobot]( https://github.com/luismesas/pydobot )说明下载Dobot-Magician Python API
- 增加move_by_angle的控制接口
  
  编辑pydobot/dobot.py文件，在move_to函数后增加move_by_angle函数定义代码：
    ````python
    def move_by_angle(self, j1, j2, j3, j4, wait=False):
        self._set_ptp_cmd(j1, j2, j3, j4, mode=PTPMode.MOVL_ANGLE, wait=wait)
    ````
- API安装
  ```shell
  python3.7.5 setup.py build
  python3.7.5 setup.py install
  ```
- 参考上述Python API的说明文档中的[Python Example](https://github.com/luismesas/pydobot#example )测试API是否安装成功
  
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - 由于机械臂Dobot-Magician的官方Python API没有提供角度控制模式，所以这里需要手动添加move_by_angle模式。

## 案例说明
### 系统数据流图
   
- 机械臂目标跟随样例的整个数据流图如下，Atlas200DK推理平台（运行环境）与Ubuntu主控平台（开发环境）通过路由器进行交互，Atlas200DK负责Yolov3目标检测模型的推理，Ubuntu主控平台通过路由器向Atlas200DK发送IntelRealsense的RGB数据，同时接收Atlas200DK的推理结果，利用推理结果对机械臂进行控制。

<div align=center><img width="750" height="190" src="https://images.gitee.com/uploads/images/2021/0310/112856_f44ecafd_5537256.png"/></div>

### 机械臂控制逻辑

<div align=center><img width="640" height="360" src="https://images.gitee.com/uploads/images/2021/0310/142505_86a67472_5537256.png"/></div>

- 问题分析
    * 分析一：机械臂的运动方式

      机械臂的J1、J2和J3轴由独立电机驱动，运动相对独立。其中J1轴控制末端中心在XY平面上的移动，若以极坐标的方式分析，可知J1轴运动只影响末端中心在XY平面的角度，控制相机朝向目标。J2轴和J3轴主要控制末端中心在XZ平面的移动，控制相机在高度上对准目标，以及相机与目标保持合适的距离。
    * 分析二：相机的位置姿态和内参外参

      根据系统示意图，双目相机安装在机械臂末端中心，末端中心的位置和姿态即为双目相机的位置和姿态。末端中心的位置可以通过机械臂SDK直接获取，可表示为$(T_x, T_y, T_z)$；通过观察，机械臂运动时，相机不存在绕X轴和Y轴的旋转，只会绕Z轴旋转，且其绕Z轴旋转的角度为机械臂J1轴的运动角度，所以其姿态可表示为$(0, 0, J_1)$。
      根据上述分析可以计算出相机的外参矩阵。  
      相机的内参矩阵可以通过Realsense的API直接获取。
    * 分析三：任务描述

      机械臂随目标在空间内移动的任务可以表述为两个子任务：
        1. 机械臂的运动总是尽量使得相机对准目标，即目标位于相机的视场（图像）中心。
        2. 目标离相机的距离大于标准距离时，机械臂要控制相机向前运动，反之亦然，即相机与目标总是保持一定的距离。

      机械臂的运动可以通过控制三个轴的角度实现，也即在拿到目标在图像坐标系中的位置以及深度数据后，J1、J2、J3三个轴要分别运动多少度，使得运动后目标位于相机的视场中心且距离合适。

- 建立数学模型
    * 坐标系转换—图像坐标系->相机坐标系->世界坐标系
        1. 相机的内参
           
            相机的内参矩阵可以通过Realsense的API直接获得，详见其API参考文档。在不考虑相机镜头畸变的情况下，内参矩阵$K$可以表示成如下形式：
           
            $$ K = \begin{bmatrix}f_x & 0 & c_x \\\\ 0 & f_y & c_y \\\\ 0 & 0 & 1 \end{bmatrix} $$
        
        2. 相机的外参
           
            根据上述相机位置的计算，可以得到其平移向量$T$：
            
            $$
            T = \begin{bmatrix} T_x \\\\ T_y \\\\ T_z \end{bmatrix}
            $$
            
            根据上述相机姿态的计算，可以得到其旋转矩阵$R$：
           
            $$
            R = \begin{bmatrix} 1 & 0 &  0 \\\\ 0 &   \cos\theta_0 & -\sin\theta_0  \\\\ 0 & \sin\theta_0& \cos\theta_0 \end{bmatrix} \cdot 
            \begin{bmatrix} \cos\theta_1 & 0 & \sin\theta_1  \\\\ 0 & 1 & 0 \\\\ -\sin\theta_1 & 0 & \cos\theta_1\end{bmatrix} \cdot 
            \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & 0 \\\\ \sin\theta_2& \cos\theta_2 & 0 \\\\ 0 & 0 & 1\end{bmatrix} 
            $$
           
            所以从相机坐标系到世界坐标系的转换为:
           
            $$
            \begin{bmatrix} x_w \\\\ y_w \\\\ z_w \end{bmatrix} = R \cdot 
            \begin{bmatrix} x_c \\\\ y_c \\\\ z_c \end{bmatrix} + T
            $$
           
        3. 图像坐标系到世界坐标系的转换
           
            完整地从图像坐标系向世界坐标系的转换过程为：
           
            $$\begin{bmatrix} x_w \\\\
                y_w \\\\
                z_w \end{bmatrix} = R \cdot 
                K \cdot \begin{bmatrix} x_p \\\\
                y_p \\\\
                1 \end{bmatrix} + T$$

    * J1轴的运动逻辑
        
        根据之前的假设，J1轴的运动问题可以在世界坐标系的XOY投影平面内进行分析:
    
        <div align=center><img width="450" height="450" src="https://images.gitee.com/uploads/images/2021/0310/113212_56e70e40_5537256.png"/></div>

        如图所示，A($x_A$, $y_A$)为目标的实际位置，B($x_B$, $y_B$)为当前相机视场中心位置（图像中心）。为使机械臂移动之后目标位于视场中心，J1轴的移动角度$\theta_1$可表示为：
        
        $$
        \theta_1 = \arccos[\frac{\overrightarrow {OA} \cdot \overrightarrow {OB}}
                                                        {\mid\overrightarrow {OA}\mid \cdot \mid\overrightarrow {OB}\mid}]
        $$
        
        即：
      
        $$
        \theta_1 = \arccos(\frac {x_A x_B + y_A y_B} {\sqrt{x_A^2 + y_A^2}\sqrt{x_B^2 + y_B^2}})
        $$
      
    * J2 & J3轴的运动逻辑
        
        <div align=center><img width="450" height="450" src="https://images.gitee.com/uploads/images/2021/0310/113332_061a0a79_5537256.png"/></div>

        建立关于$\theta_2$和$\theta_3$的方程组：
        
        $$l_2\cos(\alpha_2 + \theta_2) + l_3\cos(\alpha_3 + \theta_3) = \Delta x + l_2\cos(\alpha_2) + l_3\cos(\alpha_3)$$
        
        $$l_2\sin(\alpha_2 + \theta_2) + l_3\sin(\alpha_3 + \theta_3) = \Delta z + l_2\sin(\alpha_2) + l_3\sin(\alpha_3)$$

        求解该方程组，得到:
      
        $$\theta_3 = \arctan(\frac {R_z} {R_x}) - \arccos(\frac {l_3^2 - l_2^2 + R_x^2 + R_z^2} {2l_3\sqrt{R_x^2 + R_z^2}}) - \alpha_3$$
        $$\theta_2 = \arccos[\frac{\Delta x + l_2\cos\alpha_2 +l_3\cos\alpha_3 - l_3cos(\alpha_3 + \theta_3)}{l_2}] - \alpha_2$$
        
        其中：
        
        $$R_x = \Delta x + l_2\cos(\alpha_2) + l_3\cos(\alpha_3)$$
        
        $$R_z = \Delta z + l_2\sin(\alpha_2) + l_3\sin(\alpha_3)$$

## 案例部署
### 代码获取
- 获取源码包

   在Ubuntu主控平台中，以非root用户在命令行中执行以下命令下载源码仓:
    ```shell
    cd $HOME
    git clone https://gitee.com/ascend/samples.git
    ```

- 获取此案例中所需要的网络模型
 
    参考下表获取此应用中所用到的模型，并将其存放到开发环境普通用户下的工程目录：
    ```shell
    cd $HOME/samples/python/level3_multi_model/Robotic_Arm_Object_Following/model
    ```
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  yolov3_ascend_logo| 基于Tensorflow-YOLOV3的[Ascend图标](https://images.gitee.com/uploads/images/2021/0310/112721_0db30e79_5537256.png )检测模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3_darknet53/ATC_yolov3_darknet53_tf_xdzhangtongxue](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3_darknet53/ATC_yolov3_darknet53_tf_xdzhangtongxue )目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中已经提供了转换好的om模型，可以直接使用。
   
- 执行以下命令，将主控平台的 **samples** 目录上传到推理平台中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录Atlas200DK推理平台：
    
    ```shell
    scp -r $HOME/samples/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx 
    ```
   
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**    
    > - **xxx.xxx.xxx.xxx**为Atlas200DK的ip，在200DK与Ubuntu主控平台用USB连接时一般为192.168.1.2。

### 案例运行
- 在Atlas200DK推理平台运行可执行文件，程序将等待接收Realsense的RGB数据进行推理：

    ```shell
    cd $HOME/samples/python/level3_multi_model/Robotic_Arm_Object_Following/src 
    python3.6 object_detection.py
    ```

- 在Ubuntu主控平台运行可执行程序，发送RGB数据到Atlas200DK，并接收推理结果控制Dobot-Magician机械臂：
    
    ```shell
    cd $HOME/samples/python/level3_multi_model/Robotic_Arm_Object_Following/src
    python3.6 robotic_arm_object_following.py
    ```

### 预期结果
运行成功后，可以用[Ascend图标](https://images.gitee.com/uploads/images/2021/0310/112721_0db30e79_5537256.png )来控制机械臂的移动，预期效果如下：

<div align=center><img width="640" height="480" src="doc/demo.gif"/></div>

