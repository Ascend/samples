中文|[English](README.md)

# 级别3-行业样例

#### 目录结构与说明

本目录为行业教学样例，各文件夹对应不同种类的样例，以供用户参考，目录结构和具体说明如下。  

| 样例  | 说明  |
|---|---|
| [Robot_Play_Chess](./Robot_Play_Chess)  | 使用相机捕获对弈棋盘局面，使用Atlas200DK运行VGG16模型进行推理，在主控平台上运行对弈引擎计算出AI的走法，控制机械臂下棋， 用户使用webserver在移动端根据UI与机械臂进行对弈  |
| [Robotic_Arm_Object_Following](./Robotic_Arm_Object_Following)  |  使用Atlas200DK运行Yolov3模型，对双目深度相机给出的RGB数据流进行推理，实时检测目标在图像中的位置。并结合相机的深度数据流，控制机械臂的姿态，使得机械臂跟随目标移动 |
| [mask_rcnn_image_inpainting](./mask_rcnn_image_inpainting)  | 使用MaskRcnn和ImageInpainting模型对输入图片指定前景目标去除推理。 样例输入：待去除目标的图片。 样例输出：生成的待去除的目标的mask图片、实例分割图片和去除目标后的图片  |