[中文](README_CN.md)|English

## Third-Party Dependency Installation Guide (C++ Sample)

### Prerequisites
**1. Install the CANN package according to the official guide.**    
**2. The CANN version must be 5.0.4.alpha001 or later. If the CANN version is earlier than 5.0.4.alpha001, switch the tag and use the distribution by referring to the version description in [Ascend CANN Sample Repository](https://gitee.com/ascend/samples/tree/master).**    

### Installation Precautions
Some C++ samples in the **samples** repository use third-party dependencies such as OpenCV and FFmpeg for image processing. You need to install the third-party dependencies and configure the environment before running the samples.

The development environment and running environment are described as follows:
- **Running environment**: The running environment refers to the environment where operators, inference, or training programs run. The running environment must be on the basis of Ascend AI processors.      
- **Development environment**: Used for code development, debugging, and build. The environment can be based on Ascend AI processors or any environment that meets CANN software installation requirements.      

### Scenario Selection
Select the installation scenario based on the following description.

- [Install the development environment for the Ascend AI device and use this environment as the running environment.](catenation_environmental_guidance.md) (recommended)  
    **Advantages**: Cross-compilation is not involved. The third-party dependency installation mode is simple. You do not need to copy the compiled executable files to the running environment.  
    **Disadvantages**: The CPU usage of the running device may be low, resulting in a slow compilation speed.

- [Development environment installed on a non-AI device and running environment installed on an AI device](separate_environmental_guidance.md)    
    **Advantages**: In scenarios where the running environment is the ARM architecture, the compilation of large-scale samples in the x86 architecture development environment is fast.  
    **Disadvantages**: Cross-compilation is involved. The third-party dependency installation and sample running are complex.
