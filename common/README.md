English|[中文](README_CN.md)
中文|[English](README.md)

## Samples repository public file directory

This directory contain public file path for samples repository. Common public files have been encapsulated in terms of all samples. By doing so, duplicate sample files have been removed and repeated codes are reduced.

## File introdution

1. [presentserver](./presenterserver) + run_presenter_server.sh    
    You can refer to the sample_run.sh script in [googlenet_imagenet_video sample](../cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video) for uses of presentserver.
    - the presentserver file contain common presentserver codes for calling presentserver.     
    - run_presenter_server.sh is the common start script for presentserver.       

2. sample_common.sh     
    Common scripts used for quick integration and execution of various samples. Descriptions including common functions are as follows.
    | Name | Function | Dependent variable |
    |---|---|---|
    | find_model | input model name to check if it exists under ${ModelPath]  | ModelPath |
    | target_kernel | target compiler architecture  |  None |
    | build  | compile according to target_kernel  | TargetKernel，ScriptPath |
    | running_presenter  | use run_presenter_server.sh for C++ samples and run common presentserver  | ScriptPath，conf_file_name，common_script_dir，running_command， data_command |
    | running_presenter_python  |  use run_presenter_server.sh for python samples and run common presentserver | ScriptPath，conf_file_name，common_script_dir，running_command， data_command |
    | running | set ${running_command} before running image samples | ScriptPath，running_command |

3. testcase_common.sh + verify_result.py      
    Here are common scripts and verification files used in ST tests of various samples. ST tests are used for testing samples in an environment where development and execution are on the same device, for more information visit [st](../st).