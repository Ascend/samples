中文(./README.md)
# 内存拷贝（Host&Device）
调用AscendCL的内存管理&数据传输相关接口完成Host与Device间的数据传输。

#### 场景说明
功能：提供Host与Device间数据传输样例，帮助开发者理解相关接口与调用流程。    
输入：控制参数（是否内存复用，单次申请内存大小等）。    
输出：打屏输出device侧内存占用量及每次搬运后的内存数据。  

#### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装 |
| 芯片要求 | Ascend310/Ascend310P/Ascend910 | 芯片与产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product)|

#### 样例运行
切换到样例目录，执行如下命令：
```
cd scripts
bash sample_build.sh
cd ../out
./main --release_cycle 10 --number_of_cycles 1 --device_id 0 --memory_size 10485760 --memory_reuse 0
```
-  **--release_cycle RELEASE_CYCLE**          ：释放周期，每执行此参数数量次内存拷贝后进行一次释放，-1代表死循环
-  **--number_of_cycles NUMBER_OF_CYCLES**    ：循环周期，释放周期循环执行次数，-1代表死循环
-  **--device_id DEVICE_ID**                  ：设备ID
-  **--memory_size MEMORY_SIZE**              ：单次申请内存块大小，单位为字节
-  **--memory_reuse MEMORY_REUSE**            ：内存是否进行复用，1表进行内存复用，0表不进行内存复用

**注：由于内存不复用且一致不释放时，会在某一时刻将device内存占满，此时会循环打印内存申请失败，直至系统自动将程序杀死。这是为了体现device内存的容量上限以及让用户了解内存释放的机制，并非BUG。**

#### 查看结果
命令行可以查看每次内存拷贝前、内存拷贝后、内存回传后的内存取值及device内存状况。
![输入图片说明](image/dvpp.png)

#### 流程图&时序图


#### 主要接口说明

| 功能                | 对应ACL模块        | ACL 接口函数                      | 功能说明                                |
|--------------------|-------------------|-----------------------------------|----------------------------------------|
| 资源初始化          | 初始化             | aclInit                          | 初始化ACL配置。                         |
|                    | Device管理         | aclrtSetDevice                 | 指定用于运算的Device。                  |
|                    | Context管理        | aclrtCreateContext             | 创建Context。                          |
| 数据交互            | 内存管理            | aclrtMemcpy                    | 数据传输，Host->Device或Device->Host。  |
|                    | 内存管理            | acldvppMalloc            | 申请Device上用于媒体数据处理的内存。      |
|                    | 内存管理            | aclrtMallocHost               | 申请Host上的内存。                      |
|                    | 内存管理            | aclrtMemset                    | 初始化内存，将内存中的内容设置为指定的值。 |
|                    | 内存管理            | aclrtGetMemInfo              | 获取指定属性的内存的空闲大小和总大小。     |
| 资源释放            | 内存管理            | acldvppFree              | 释放Device上用于媒体数据处理的内存。      |
|                    | 内存管理            | aclrtFreeHost                 | 释放Host上的内存。                       |
|                    | Context管理         | aclrtDestroyContext           | 销毁Context。                           |
|                    | Device管理          | aclrtResetDevice              | 复位当前运算的Device，回收Device上的资源。 |
|                    | 去初始化            | aclFinalize                     | 实现ACL去初始化。                        |

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。