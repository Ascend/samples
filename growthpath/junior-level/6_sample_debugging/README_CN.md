中文|[English](./README.md)
# 样例调试

### <a name="step_1"></a> 本章学习目标
- 了解日志处理流程
- 了解常见问题及处理方法
- 具备初步问题处理与分析能力

### <a name="step_2"></a> 日志说明
日志主要用于记录系统的运行过程及异常信息，帮助快速定位系统运行过程中出现的问题以及开发过程中的程序调试问题。在应用调试过程中，经常会查看日志并进行问题定位。    
日志级别等级由低到高顺序：DEBUG < INFO < WARNING < ERROR，级别越低，输出日志越详细。
<table>
<tr><td width="25%"><b>日志类型</b></td><td width="25%"><b>日志级别</b></td><td width="50%"><b>定义</b></td></tr>
<tr><td rowspan="5" valign="top">运行日志</td><td>ERROR</td><td>一般错误级别。该级别的日志记录了如下错误：     

· 非预期的数据或者事件。   
· 影响面较大且模块内部能够处理的错误。     
· 限制在模块内的错误。    
· 对其他模块有轻微影响的错误，例如统计任务创建失败。      
· 引起调用失败的错误。      
· 在业务逻辑错误的情况下记录错误状态的信息及造成错误的可能原因。</td></tr>
<tr><td>WARNING</td><td>警告级别。记录系统和预期的状态不一致，但不影响整个系统运行的信息。</td></tr>
<tr><td>INFO</td><td>正常级别。记录系统正常运行的信息。</td></tr>
<tr><td>DEBUG</td><td>调试级别。该级别的日志记录了调试信息，便于开发人员或维护人员定位问题。</td></tr>
<tr><td>NULL</td><td>NULL级别。不输出日志。</td></tr>
</tr>
<tr><td rowspan="1" valign="top">Trace日志</td><td>-</td><td>记录模块运行全生命周期中模块流程交互、状态迁移时的信息。</td></tr>
<tr><td rowspan="1" valign="top">Oplog日志</td><td>-</td><td>记录进程任务创建或销毁资源时的日志。</td></tr>
<tr><td rowspan="1" valign="top">EVENT日志</td><td>-</td><td>记录系统关键事件日志，例如：整网运算启动、完成、异常终止，内存不足，单板温度过高等信息。</td></tr>
</table>

系统记录的日志一般都具有特殊结构，样例如下：
```
[ERROR] TEFUSION(12940,atc):2021-10-17-05:54:07.599.074 [tensor_engine/te_fusion/pywrapper.cc:33]InitPyLogger Failed to import te.platform.log_util
```
如上就是一行典型的ERROR日志，遵循的格式如下：
```
[Level] ModuleName(PID,PName):DateTimeMS [FileName:LineNumber]LogContent
```
字段说明如下：
| 字段  | 说明  |
|---|---|
| Level  | 日志类型。运行日志存在5种日志类型：<br>ERROR、WARNING、INFO、DEBUG、EVENT。  |
| ModuleName  | 产生日志的模块的名称。  |
| PID  |  进程ID。 |
| PName  | 进程名称。  |
| DateTimeMS  | 日志打印时间，格式为：<br>yyyy-mm-dd-hh:mm:ss.SSS。  |
| FileName:LineNumber  | 调用日志打印接口的文件及对应的行号。  |
| LogContent  | 各模块具体的日志内容。  |


### <a name="step_3"></a>日志获取
了解了日志的级别和内容后，接下来就要知道从哪里可以获取到日志。
1. 打屏显示。    
   样例运行过程中，可以通过设置环境变量的方式，让日志直接打印到终端stdout中。设置的环境变量如下：
   ```
   export ASCEND_SLOG_PRINT_TO_STDOUT=1
   ```
   当日志都打屏显示到终端后，可以通过重定向的方式，将所有内容重定向到文件，再进行分析。如：
   ```
   ./main > log.txt
   ```
2. 获取后台日志文件。     
   日志文件默认路径为：$HOME/ascend/log。（注意：其中Atlas200DK默认路径为：/var/log/npu/slog。）     
   除Atlas200DK场景外，可通过设置以下环境变量，修改日志文件的落盘路径：
   ```
   export ASCEND_PROCESS_LOG_PATH=$HOME/xxx
   ```
   但是需要保证，设置的路径为任意有读写权限的目录。
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/log_path.png)

### <a name="step_4"></a>日志操作    
   当需要获取更详细日志进行问题分析，或减少日志打印提升性能时；可通过设置如下环境变量调整日志打印。
   ```
   export ASCEND_GLOBAL_LOG_LEVEL=1
   ```
   该环境变量取值如下：
   - 0：对应DEBUG级别。
   - 1：对应INFO级别。
   - 2：对应WARNING级别。
   - 3：对应ERROR级别。
   - 4：对应NULL级别，不输出日志。
   - 其他值为非法值。

### <a name="step_5"></a>典型场景
以下介绍一些典型的问题。   
1. 接口调用逻辑问题        
   - 内存申请/释放接口不成对。   
     问题现象：内存占用率持续上升，出现内存泄漏。
     内存上涨如下图所示。   
     ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/memory_up.png)  
     应用报错如下图所示。    
     ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/samples_failed.png)  
     日志报错如下图所示。   
     ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/sample_failed_log.png)  
     一般原因：用户代码中只申请内存，不释放内存，导致内存泄漏的现象    
     解决思路：使用相应资源后需及时释放。    
     补充说明：申请与释放必须成对出现，确保一一对应。例如aclrtMalloc与aclrtFree，aclrtMallocHost与aclrtFreeHost、aclrtCreateStream与aclrtDestroyStream等。  

   - 资源释放冗余。   
     问题现象：device报错，资源释放失败，如下device-0_2020090421508557.log中报错     
     ```
     [ERROR] TSCH(-1,null):2020-09-04-21:51:13.452.762 63696 (cpuid:0) task_scheduler_engine.c:707 proc_model_stream_unbind: unbind model stream failed, stream is running. stream->model_id=512, model_id=512, task.sq_id=514, task.task_id=3
     ```        
     一般原因：应用进程异常时，用户捕获进程异常的信号并主动完成清理，用户主动清理会影响到系统的资源释放流程。      
     解决思路：用户无需关注进程异常退出信号      
     补充说明：Host侧内核态驱动会自动检测并发起对应进程Device侧资源释放的流程，不需要用户主动清理。

   - 调用异步接口后未同步。    
     问题现象：样例报错或推理结果错误。      
              正常推理结果如下所示。
              ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/inference_correct.png)    
              未同步推理结果如下所示。
              ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/inference_wrong.png)  
     一般原因：调用异步接口成功仅表示任务下发成功，不表示任务执行成功，无法在调用异步接口后直接获取结果数据。     
     解决思路：异步场景下及时执行流同步。    
     补充说明：调用异步接口后，需调用同步等待接口aclrtSynchronizeStream，确保Device侧任务执行完成，能获取结果数据。

2. 输入相关问题     
   - 输入图片格式不支持，JPEG图片解码失败    
     问题现象：JPEG解码报错    
     ```
     Unsupported subsample format, just support jpeg with YUV 444 440 422 420 400
     do not support progressive mode
     do not support arithmetic code, support huffman code only
     ```    
     一般原因：JPEG图片解码只支持huffman编码(colorspace: yuv, subsample: 444/440/422/420/400 )，不支持算术编码，不支持渐进编码，不支持jpeg2000格式。    
     解决思路：使用正确的输入图片格式     
     补充说明：选择硬件范围内的图片格式，或通过DVPP多个功能的组合，例如JPEGD解码+VPC图片格式转换，输出符合要求的图片格式。     
   
   - 输入图片异常，JPEG图片解码失败。    
     问题现象：JPEG解码报错     
     ```
     EOI segment of the stream is invalid
     EOI segment of the stream is invalid, it should be FFD9. Try software decoding
     ```
     一般原因：图像数据不完整，缺失最后的EOI结束符（标记码FF D9）。如果原图像数据完整，可能数据在传输过程中存在损坏，需要在调用JPEG解码接口前，通过fwrite函数将传输后的码流保存下来，与原图进行二进制比较，定位传输过程中的数据缺失问题。    
     解决思路：检查异常图片    
     补充说明：正常JPEG图片最后应该由标记码d9ff结束。      
     ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/d9ff.png)