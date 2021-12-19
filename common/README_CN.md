中文|[English](README.md)

## samples仓公共文件目录

该目录为samples仓公共文件路径，针对所有样例共用的公共文件进行封装，已达到去除各样例的重复文件，减少仓库重复代码的目的。

## 文件介绍

1. [presentserver](./presenterserver) + run_presenter_server.sh    
    关于presentserver的使用可以参考[googlenet_imagenet_video样例](../cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video)中的sample_run.sh脚本。
    - presentserver文件夹为公共presentserver代码，提供给各使用presenter能力的样例调用。     
    - run_presenter_server.sh文件为presenterserver的公共启动文件。       

2. sample_common.sh     
    各样例快速部署和运行所使用的公共脚本文件，包含公共函数说明如下表所示。
    | 函数名 | 作用 | 依赖变量 |
    |---|---|---|
    | find_model | 输入模型名称，查找${ModelPath]下该模型是否存在  | ModelPath |
    | target_kernel | 用户选择目标编译器的CPU架构  |  无 |
    | build  | 根据target_kernel结果编译样例  | TargetKernel，ScriptPath |
    | running_presenter  | C++样例调用run_presenter_server.sh脚本，运行公共presentserver  | ScriptPath，conf_file_name，common_script_dir，running_command， data_command |
    | running_presenter_python  |  python样例调用run_presenter_server.sh脚本，运行公共presentserver。 | ScriptPath，conf_file_name，common_script_dir，running_command， data_command |
    | running | 图片样例运行接口，需要提前设置好${running_command} | ScriptPath，running_command |

3. testcase_common.sh + verify_result.py      
    各样例ST测试样例所使用的公共脚本文件和校验文件。ST测试样例主要用于合设环境测试样例的基本能力，使用者无需关心，详情可以在[st](../st)目录中了解。