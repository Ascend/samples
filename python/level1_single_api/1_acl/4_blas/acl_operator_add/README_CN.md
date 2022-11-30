# 实现矩阵-矩阵相加运算

## 功能描述

此样例实现了对自定义算子的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过ACL加载单算子模型文件进行运行。  
该实现矩阵-矩阵相加的运算示例为： C = A + B，其中A、B、C都是8*16的矩阵，类型为int32，矩阵加的结果是一个8\*16的矩阵。

## 目录结构

如下为模型文件转换后的示例目录结构， op_models文件夹是转换后生成的。

```
acl_operator_add
├──src
│ ├── acl_execute_add.py //运行文件
│ └── constant.py //常量定义
└── test_data
  ├── config
  │ ├── acl.json //系统初始化的配置文件
  │ └── add_op.json //矩阵相加算子的描述信息
  └── op_models
    └── 0_Add_3_2_8_16_3_2_8_16_3_2_8_16.om //矩阵相加算子的模型文件
```

## 主要接口

| 功能            | 对应ACL模块    | ACL 接口函数               | 功能说明                                |
|----------------|----------------|---------------------------|-----------------------------------------|
| 资源初始化      | 初始化         | acl.init                  | 初始化ACL配置。                          |
|                | Device管理     | acl.rt.set_device         | 指定用于运算的Device。                   |
|                | Context管理    | acl.rt.create_context     | 创建Context。                           |
|                | Stream管理     | acl.rt.create_stream      | 创建Stream。                            |
|                | 算子加载与执行  | acl.op.set_model_dir      | 加载模型文件的目录。                     |
| 数据后处理      | 算子加载与执行  | acl.op.create_attr        | 创建aclopAttr类型的数据。                |
|                | --             | acl.create_tensor_desc    | 创建aclTensorDesc类型的数。             |
|                | --             | acl.get_tensor_desc_size  | 获取tensor描述占用的空间大小。           |
|                | --             | acl.create_data_buffer    | 创建aclDataBuffer类型的数据。           |
| 数据交互        | 内存管理       | acl.rt.memcpy             | 数据传输，Host->Device或Device->Host。  |
|                | 内存管理        | acl.rt.malloc             | 申请Device上的内存。                    |
|                | 内存管理        | acl.rt.malloc_host        | 申请Host上的内存。                      |
| 单算子推理      | 算子加载与执行  | acl.op.execute            | 异步加载并执行指定的算子。                |
| 公共模块        | --             | acl.util.ptr_to_numpy     | 指针转numpy类型数据。                    |
|                | --             | acl.util.numpy_to_ptr     | numpy类型数据转指针。                    |
|                | --             | acl.util.bytes_to_ptr     | bytes类型数据转指针。                    |
|                | --             | acl.util.ptr_to_bytes     | 指针转bytes类型数据。                    |
| 资源释放        | 内存管理       | acl.rt.free               | 释放Device上的内存。                     |
|                | 内存管理       | acl.rt.free_host           | 释放Host上的内存。                      |
|                | Stream管理     | acl.rt.destroy_stream      | 销毁Stream。                            |
|                | Context管理    | acl.rt.destroy_context     | 销毁Context。                           |
|                | Device管理     | acl.rt.reset_device        | 复位当前运算的Device，回收Device上的资源。|
|                | 去初始化        | acl.finalize              | 实现ACL去初始化。                        |

## <span id = "env">环境变量配置 </span>

在安装完CANN软件包之后，请务必自行配置pyACL相关的环境变量，否则，将无法正常“import acl”。

1. 若环境中安装了cann-toolkit软件包：
```
# 以root用户安装toolkit包
. /usr/local/Ascend/ascend-toolkit/set_env.sh
# 以非root用户安装toolkit包
. ${HOME}/Ascend/ascend-toolkit/set_env.sh
```
2. 若环境中安装了cann-nnrt软件包：
```
# 以root用户安装nnrt包
. /usr/local/Ascend/nnrt/set_env.sh
# 以非root用户安装nnrt包
. ${HOME}/Ascend/nnrt/set_env.sh
```
3. 若环境中安装了cann-nnae软件包：
```
# 以root用户安装nnae包
. /usr/local/Ascend/nnae/set_env.sh
# 以非root用户安装nnae包
. ${HOME}/Ascend/nnae/set_env.sh
```

-   **Atlas 500 智能小站：**
    1.  配置PYTHON环境变量，将“/home/data/miniD/driver/lib64”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/home/data/miniD/driver/lib64:$PYTHONPATH
        ```

    2. 配置LD\_LIBRARY\_PATH，将“/home/data/miniD/driver/lib64”加入到LD_LIBRARY_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/home/data/miniD/driver/lib64:$LD_LIBRARY_PATH
        ```

-   **Atlas 200 AI加速模块（ RC场景）：**
    1.  安装minirc版本的runtime包，以安装在默认路径“/usr/local/Ascend”为例：配置LD\_LIBRARY\_PATH，将“/usr/local/Ascend/runtime/lib64”加入到LD\_LIBRARY\_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/usr/local/Ascend/runtime/lib64:$LD_LIBRARY_PATH
        ```

    2.  获取aarch64架构的cann-nnrt软件包，在服务器上解压，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-cann-nnrt_xxx_linux-aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压目标路径下找到“run_package/Ascend-pyACL-xxx-linux.aarch64.run”，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-pyACL-xxx-linux.aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压路径下找到“python/site-packages/acl/acl.so”，将acl.so上传至200RC上，例如，上传至“ /usr/local/Ascend/pyACL/python/site-packages/acl”目录下，配置PYTHON环境变量，将“ /usr/local/Ascend/pyACL/python/site-packages/acl”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/usr/local/Ascend/pyACL/python/site-packages/acl:$PYTHONPATH
        ```

## 运行应用

1.  模型转换。
    1.  以HwHiAiUser（运行用户）登录开发环境。
    2.  参见《[CANN 开发辅助工具指南](https://support.huawei.com/enterprise/zh/doc/EDOC1100206690?idPath=23710424%7C251366513%7C22892968%7C251168373)》中的“ATC工具使用指南”章节中的ATC工具使用环境搭建，获取ATC工具并设置环境变量。

    3.  在acl_operator_add/test_data路径下执行如下命令，生成单算子模型文件。 Ascendxxx为使用的昇腾AI处理器型号，请用户自行替换。
        ```
        atc --singleop=config/add_op.json --soc_version=Ascendxxx --output=op_models
        ```
        -   --output参数：生成的*.om文件存放在“ ./op_models”目录下。
        -   使用atc命令时用户需保证对acl_operator_add/test_data目录有写权限。

        模型转换成功后，会生成如下文件：  
        在当前目录的op_model目录下生成单算子的模型文件0_Add_3_2_8_16_3_2_8_16_3_2_8_16.om。

        -   算子模型文件的命名规范为：序号+opType+输入的描述(dateType_format_shape)+输出的描述。

    4.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境。

2.  运行应用
    1.  登录运行环境。
    2.  参照[环境变量配置](#env)完成运行环境的配置。
    3.  在acl_operator_add/test_data路径下执行如下命令：
        ```
        python3 ../src/acl_execute_add.py
        ```
