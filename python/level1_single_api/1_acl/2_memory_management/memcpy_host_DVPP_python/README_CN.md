中文(./README.md)
# 数据传输（Host&Device(DVPP)）
调用pyacl的内存管理&数据传输相关接口完成Host与Device(DVPP)间的数据传输。

#### 场景说明
功能：提供Host与Device(DVPP)间数据传输样例，帮助开发者理解相关接口与调用流程。    
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
python3 memcpy_host_dvpp.py --release_cycle 10 --number_of_cycles 10 --device_id 0 --memory_size 10485760 --memory_reuse
```
-  **-h, --help**                             ：展示帮助信息
-  **--release_cycle RELEASE_CYCLE**          ：释放周期，每执行此参数数量次内存拷贝后进行一次释放，-1代表死循环，默认值为-1
-  **--number_of_cycles NUMBER_OF_CYCLES**    ：循环周期，释放周期循环执行次数，-1代表死循环，默认值为1
-  **--device_id DEVICE_ID**                  ：设备ID，默认值为0
-  **--memory_size MEMORY_SIZE**              ：单次申请内存块大小，单位为字节，默认值为10485760
-  **--memory_reuse MEMORY_REUSE**            ：内存是否进行复用，如不带此参数则不复用

**注：如不带参数仅执行python3 memcpy_host_dvpp.py，所有参数会使用如上所示的默认值，由于内存不复用且一致不释放，会在某一时刻将device内存占满，此时会循环打印内存申请失败，直至系统自动将程序杀死。这是为了体现device内存的容量上限以及让用户了解内存释放的机制，并非BUG。**
![输入图片说明](../../../picture/memcpy_host_dvpp_python_1.png)

#### 查看结果
命令行可以查看每次内存拷贝前、内存拷贝后、内存回传后的内存取值及device内存状况。
![输入图片说明](../../../picture/memcpy_host_DVPP_python_2.png)

#### 流程图&时序图
![输入图片说明](../../../picture/memcpy_host_dvpp_python_3.jpg)

#### 主要接口说明

| 功能                | 对应ACL模块        | ACL 接口函数                      | 功能说明                                |
|--------------------|-------------------|-----------------------------------|----------------------------------------|
| 资源初始化          | 初始化             | acl.init                          | 初始化ACL配置。                         |
|                    | Device管理         | acl.rt.set_device                 | 指定用于运算的Device。                  |
|                    | Context管理        | acl.rt.create_context             | 创建Context。                          |
| 数据交互            | 内存管理            | acl.rt.memcpy                    | 数据传输，Host->Device或Device->Host。  |
|                    | 内存管理            | acl.media.dvpp_malloc            | 申请Device上用于媒体数据处理的内存。      |
|                    | 内存管理            | acl.rt.malloc_host               | 申请Host上的内存。                      |
|                    | 内存管理            | acl.rt.memset                    | 初始化内存，将内存中的内容设置为指定的值。 |
|                    | 内存管理            | acl.rt.get_mem_info              | 获取指定属性的内存的空闲大小和总大小。     |
| 公共模块            | --                 | acl.util.numpy_to_ptr            | numpy类型数据转指针。                    |
|                    | --                 | acl.util.bytes_to_ptr            | bytes类型数据转指针。                    |
| 资源释放            | 内存管理            | acl.media.dvpp_free              | 释放Device上用于媒体数据处理的内存。      |
|                    | 内存管理            | acl.rt.free_host                 | 释放Host上的内存。                       |
|                    | Context管理         | acl.rt.destroy_context           | 销毁Context。                           |
|                    | Device管理          | acl.rt.reset_device              | 复位当前运算的Device，回收Device上的资源。 |
|                    | 去初始化            | acl.finalize                     | 实现ACL去初始化。                        |

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。