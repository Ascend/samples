# 棋盘理解开发文档

## 1. 总体功能

棋盘理解模块主要负责通过Camera采图经过处理后送入Atlas 200DK进行推理，从而完成对棋盘状态的解析。

## 2. 概要设计

### 2.1 结构设计

棋盘理解被设计为2个关联的类。一个类是棋盘理解本身，用来实现棋盘理解的各种功能；另一个类是200DK上的推理分类器，仅仅用来实现分类功能。

#### 2.1.1 class ChessStatusPerception

棋盘理解类主要包含畸变矫正、圆检测、分类、整理信息功能，使用成员函数process将各种功能统一。

Process: 整合全部流程，封装好可以直接调用来得到棋盘理解的结果。

Undistortion: 图像去畸变，其中去畸变的参数详见去畸变目录下文档。**非必要请不要动相机的焦距**（否则要重新测相机内参）

GetCirclesInfo: 圆检测，用来得到棋盘上棋子的圆心。注意检测时候的光源配置，请保证光照均匀

Classification: 分类，根据全部的圆心列表，对原图进行crop切片得到每一个小棋子的图片56x56，送进分类器分类。

BoardCalib: 标定函数，用来将去畸变后的图像通过透视变换到理想棋盘坐标系下。

#### 2.1.2 class Classify

分类器类主要包括atlas 200DK上配置的分类器模型参数。包含了加载模型、图像预处理、推理、后处理等过程，通过process整合功能封装。

init： 初始化函数，加载模型

pre_process: 图像预处理，将输入的图像resize到模型需要的输入大小。

inference: 分类器执行推理过程。

post_process: 图像后处理，整理推理得到的分类结果。

process: 整合上述全部功能，封装以备调用。

### 2.2 接口设计

主要介绍一下棋盘理解和中心控制模块的联系。

在棋盘理解模块的main函数中，通过设定的batchsize来分段接收一张图片，并调用棋盘理解类perception来实现全部功能；

返回的信息包括棋盘理解得到的理想棋子坐标信息ideal和真实棋子坐标信息real，中间用符号\&来连接。信息由[x, y, class]构成，打包发送的数据是：


"[x_ideal, y_ideal, c]#[x, y, c]#...[]&[x_real, y_real, c]#[x, y, c]#...[]"

注意这里头部没有井号，和发送给webserver端的信息不同，发送给webserver之前会在这段信息之前加上一个井号。（设计遗留缺陷）


## 3. 调用方法

棋盘理解启动在200DK上，启动前请检查atlas相关包安装路径并将它写进main.py关联路径：
```python
sys.path.append("/home/HwHiAiUser/Workspace/s00522947/common/")

```
检查main.py分类器的om模型是否正确：
```python
MODEL_PATH = "/home/HwHiAiUser/Workspace/s00522947/chinese_chess/cchess/model/chess_ckpt_0804_vgg_99.om"

```
检查棋盘理解类文件ChessStatusPerception.py中的标定位置是否正确：
```python
    def BoardCalib(self, image, circlesInfo):
        # 标定四个角的坐标 
        corners = np.float32([[72, 145], [63, 501], [385, 146], [386, 506]])
        ## ...
        pass
```
检查棋盘理解类文件ChessStatusPerception.py中的ip和端口是否和中心控制保持一致，保证连入同一个网：
```bash
$ifconfig -a
```

一切检查都OK之后，进入目录运行main.py挂起棋盘理解服务：
```bash
$python3 main.py
```

