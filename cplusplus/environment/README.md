English|[中文](README_CN.md)

## Third-party dependency installation guide（C++ sample）

### Precondition
**Install CANN package according to official documents.**   

### Installation note
Some python samples in the samples repository use third-party dependencies such as opencv and ffmpeg for image processing, so you need to install third-party dependencies and configure the environment according to the instructions in this article before running. 
- **Development environment：** Mainly used for coding, compiling and testing.    
    （Scenario 1）Install development environment on Ascend AI device which can be used as operation environment, application execution and training script migration, testing and development.     
    （Scenario 2）Install development environment on non-Ascend device which can only be used for coding, compiling, and other activities that do NOT rely on Ascend devices (such as ATC model convert, operator, application inference and other plainly code development).    
- **Operation environment：** Migrate training scripts, code and test on applications developed by operation user on Ascend AI device.   

### Scenario selection
Select the corresponding scenario according to the following description.

- [Ascend AI device as development, operation environment and third-party dependency installation in the operation scenario](./catenation_environmental_guidance_CN.md) **（recommended）**.   
    **Pros：** Doesn't involve cross-compilation, easy third-party installation, compilation and execution together and no need to copy compiled executable file to operation environment.   
    **Cons：** The CPU in development environment might be slow which results in slow compilation.

- [Install third-party dependency in development environment on non-Ascend Ai device](./separate_environmental_guidance_CN.md).     
    **Pros：** Aimed at arm operation environment, using x86 architecture for large sample compilation is quick.   
    **Cons：** Involve cross-compilation, third-party dependency installation and sample execution might be complicated. 


# C++ environment preparation and dependency installation

### Introduction
Introduction


### Steps
Please perform the following four steps in sequence, please click the corresponding link to install according to the device type.

- for_atlas200dk  

    [1.Basic environment configuration](./prepare_ENV/README_200DK_EN.md)： configure the basic environment, including the sudo permission, apt source, environment variables and deploying the Media Module  

    [2.Install ffmpeg and opencv](./opencv_install/README_200DK_EN.md)： install ffmpeg and opencv  

    [3.Install acllite](./acllite_install/README_200DK_EN.md)： The Atlasutil library encapsulates the ACL portion of the API.   

    [4.Install Presenter Agent](./presenteragent_install/README_200DK_EN.md)： The Presenter Agent provides a series of APIs that users can call to push media messages to the presenter server.  

- for_atlas300

    [1.Basic environment configuration](./prepare_ENV/README_300_EN.md)： configure the basic environment, including the sudo permission, apt source and environment variables  

    [2.Install ffmpeg and opencv](./opencv_install/README_300_EN.md)： install ffmpeg and opencv  

    [3.Install acllite](./acllite_install/README_300_EN.md)： The Atlasutil library encapsulates the ACL portion of the API.   

    [4.Install Presenter Agent](./presenteragent_install/README_300_EN.md)： The Presenter Agent provides a series of APIs that users can call to push media messages to the presenter server.  

