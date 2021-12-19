English|[中文](README_CN.md)

## Sample testcase

This testcase directory is used for verifying sample functions. The corresponding testcase needs to be in the same directory when sample is submitted.
Testcase is mainly used for auto-testing on environments where development and execution are on the same device and has no impact on the sample itself. 
Refer to readme when using the sample.
## Introduction

Testcase script is in the ST file in samples directory, test script for C++ projects is in cplusplus directory wheres test script for python projects is in python directory.
Name the script with testcase_ + "project name" + .sh in which "project name" corresponds with the directory name superior to the src file.    
Take the C++ YOLOV3_coco_detection_picture sample for example.     
Sample src directory is：samples/cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture/src     
So testcase directory should be：st/cplusplus/testcase_YOLOV3_coco_detection_picture.sh
 
> **Description：** 
>- Every entry script corresponds to two device forms and the current project should adapt to Atlas200dk and A300-3010.   
>- Each device form corresponds to one or more software versions and the current project should at least adapt to one version. Currently there is only one test environment: CANN 5.0.3.alpha005 on Atlas200dk and A300-3010.
>- Note: blacklist_version.conf contains versions that does NOT run testcase. Example: testcase_YOLOV3_coco_detection_picture: 5.0.3.alpha005

## testcas_common.sh function description

| Name  | Function  | Dependent variable  |
|---|---|---|
| downloadDataWithVerifySource  | Download data and verification file, calling once will download single data and verification file | project_path，data_name，data_source，verify_source，verify_name  |
| downloadData2  | Download single data file  | project_path，data_source，data_name2 |
| downloadVerifySource2  | Download single verification file  | project_path，verify_source，verify_name2  |
| downloadOriginalModel  | Download model file for model convert  | project_path，(caffe_prototxt，caffe_model)/tf_model/onnx_model/mindspore_model/json_model，aipp_cfg|
| modelconvert  | Convert model  | project_name，model_name，model_atc  |
| buildproject  | Compile codes  | project_path  |
| run_picture  | Run C++ image sample and image verification  | project_path，run_command  |
| run_picture_python  | Run python image sample and image verification  | project_path，run_command  |
| run_presenter | Run C++ presentserver sample | common_script_dir，script_path，conf_file_name，project_path，project_name，run_command |
| run_presenter_python | Run python presentserver sample | common_script_dir，script_path，conf_file_name，project_path，run_command |
| run_md5 | Generate C++ sample execution and md5 verification file | project_path，run_command，verify_name |
| run_md5_python | Generate python sample execution and md5 verification file | project_path，run_command，verify_name |
| run_txt | Generate txt sample execution file and txt comparison file | project_path，run_command，txt_file，verify_name |

