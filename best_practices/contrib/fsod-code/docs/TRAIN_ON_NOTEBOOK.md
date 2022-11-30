# Train on Notebook
## 1 创建Notebook环境
我们测试训练使用的环境为ModelArts提供的Notebook里的Jupyterlab环境；JupyterLab是一个交互式的开发环境，是Jupyter Notebook的下一代产品，可以使用它编写Notebook、操作终端、编辑MarkDown文本、打开交互模式、查看csv文件及图片等功能。JupyterLab具有和Jupyter Notebooks一样的组件，但支持更加灵活和更加强大的项目操作方式。


下面介绍如何从运行中的Notebook实例打开JupyterLab。
1. 登录ModelArts管理控制台（注意选择 **北京四** 地区），在左侧菜单栏中选择“镜像管理”，进入镜像管理界面，点击注册按钮，进行自定义镜像注册；
> 1 使用的镜像地址为：swr.cn-north-4.myhuaweicloud.com/ascend-share/5.1.rc2.alpha005_torch-ascend910-cp37-euleros2.8-aarch64-training:1.15.0-21.0.2_0602_to

> 2 选择ARM架构；

> 3 增加ASCEND类型。


2. 登录ModelArts管理控制台，在左侧菜单栏中选择“开发环境 > Notebook”，进入新版Notebook管理页点击创建，选用刚刚注册的自定义镜像。


3. 选择状态为“运行中”的Notebook实例，单击操作列的“打开”，访问JupyterLab。

4. 进入JupyterLab页面后，自动打开Launcher页面，如下图所示。您可以使用开源支持的所有功能，详细操作指导可参见[JupyterLab官网文档](https://jupyterlab.readthedocs.io/en/stable/)。


## 2 克隆项目代码
打开一个终端terminal，运行下列git clone命令进行项目代码克隆（若无法使用git，请直接下载代码zip包上传）：

```bash
git clone https://gitee.com/XiongfeiBai/samples
```


## 3 准备训练数据集
进入代码文件夹

```bash
cd samples/best_practices/contrib/fsod-code/
```

1. 创建并进入数据集文件夹datasets：

```bash
mkdir datasets
cd datasets
```

2. 使用moxing进行obs数据集下载,在命令行输入python3进入python终端（操作完成后退出）：

```python
import moxing as m
m.file.copy_parallel("obs://few-shot-bj4/VOC-custom.zip", "VOC-custom.zip")
quit()
```

3. 在datasets文件夹内解压数据集：


```bash
unzip VOC-custom.zip
cd ..
```

4. 配置所需的环境：

```bash
bash setup.sh
```

5. 运行训练脚本：

```
bash train.sh
```




