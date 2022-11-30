# 泊松融合（Possion Fusion）

简介：对于某些工业数据集如钢铁表面缺陷，表面刮痕，焊缝缺陷等，若正常样本的数量较多，且缺陷样本的数量很少，出现

正负样本不平衡的情况，可考虑使用泊松融合的方法扩充缺陷样本数量，以便于进行后续的训练。

## 1. 缺陷提取

缺陷提取的目的主要是为了得到缺陷区域的二值掩膜（mask）,后续根据该mask 中的白色区域决定参与融合的缺陷区域大小和位置。

以下提供三种方法获取相应的mask:

a:直接使用裁剪出缺陷区域，并以该整个区域作为融合区域（融合效果一般，但操作简单）

		1. 将正常样本放入PossionFusion/normal_data文件夹中
		1. 将缺陷样本放入PossionFusion/defect_data中

![image-20220922164453020.png](https://gitee.com/XiongfeiBai/fsod-code/raw/master/image-20220922164453020.png)

b:对缺陷区域进行二值分割，得到较为精细的mask。

  		1. 将正常样本放入PossionFusion/normal_data文件夹中
  		2. 将缺陷样本放入PossionFusion/defect_data文件夹中
  		3. 将mask放入PossionFusion/mask文件夹中

![image-20220922164453020.png](https://gitee.com/XiongfeiBai/fsod-code/raw/master/image-20220922164236344.png)

c.若既想要较为准确的mask,但通过二值分割的方法很难得到较好的分割结果，可参考该工程化方法，具体如下：

  		1. 将正常样本放入PossionFusion/normal_data文件夹中
  		2. 利用labelme对缺陷区域进行标注，将得到的.json文件放入PossionFusion/json_data

## 2. 缺陷融合

1.按照上述三种方法的一种放好相应的数据

2.修改PossionFusion/args.py中的配置，根据所选的方法，修改method=1（2或3），fusion-only设置为False

3.运行一次run.py，后将fusion-only设置为True,不断运行run.py即可

4.最终生成的融合图片存放在gen_data文件夹中