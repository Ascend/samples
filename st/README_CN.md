中文|[English](README.md)

## 样例测试用例

该目录为样例测试用例目录，主要用于样例的功能性验证。提交样例时需要保证该目录下有对应的测试样例。
测试用例主要用于合设环境上的自动化测试，对样例本身无任何影响，如果使用样例请参考各样例README进行使用参考。

## 编写说明

samples的目录下st的文件夹中放置用户testcase测试脚本,cplusplus下放置C++工程的测试脚本，python文件夹下放置python工程的测试脚本。命名规则要求为testcase_ + "工程名称" + .sh，其中，工程名称为对应工程的src文件夹上一级目录名。    
以C++ YOLOV3_coco_detection_picture样例为例。     
样例src目录为：samples/cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture/src     
因此testcase的脚本为：st/cplusplus/testcase_YOLOV3_coco_detection_picture.sh
 
> **说明：** 
>- 每一个入口脚本对应两种设备形态，要求当前工程适配这两种设备形态：Atlas200dk，A300-3010。   
>- 每一个设备形态对应一种或多种版本，要求当前工程至少适配一种版本。当前测试环境只有1个版本，CANN 5.0.3.alpha005的Atlas200dk和A300-3010。
>- 注意：blacklist_version.conf中的内容为黑名单版本，即不会在该版本上跑该testcase。例如 testcase_YOLOV3_coco_detection_picture: 5.0.3.alpha005

## testcas_common.sh函数说明

| 函数名称  | 作用  | 依赖变量  |
|---|---|---|
| downloadDataWithVerifySource  | 下载数据文件和验证文件，调用一次下载一个数据文件和一个数据文件  | project_path，data_name，data_source，verify_source，verify_name  |
| downloadData2  | 下载单个数据文件  | project_path，data_source，data_name2 |
| downloadVerifySource2  | 下载单个验证文件  | project_path，verify_source，verify_name2  |
| downloadOriginalModel  | 下载原始模型，为模型转换做准备  | project_path，(caffe_prototxt，caffe_model)/tf_model/onnx_model/mindspore_model/json_model，aipp_cfg|
| modelconvert  | 模型转换  | project_name，model_name，model_atc  |
| buildproject  | 代码编译  | project_path  |
| run_picture  | C++图片样例运行及图片校验  | project_path，run_command  |
| run_picture_python  | python图片样例运行及图片校验  | project_path，run_command  |
| run_presenter | C++ presentserver样例运行 | common_script_dir，script_path，conf_file_name，project_path，project_name，run_command |
| run_presenter_python | python presentserver样例运行 | common_script_dir，script_path，conf_file_name，project_path，run_command |
| run_md5 | C++ 生成文件样例运行及md5校验文件 | project_path，run_command，verify_name |
| run_md5_python | python 生成文件样例运行及md5校验文件 | project_path，run_command，verify_name |
| run_txt | 生成txt样例运行及txt文件比对 | project_path，run_command，txt_file，verify_name |
