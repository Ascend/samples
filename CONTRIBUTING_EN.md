English|[中文](README_CN.md)

 **Introduction**

Welcome to use Ascend Samples!

 **Contribution Requirements**

The submitted project files shall contain at least the source code, readme file, and build and run scripts.

Sign the contribution license agreement (CLA) before your submission. [Link](https://clasign.osinfra.cn/sign/Z2l0ZWUlMkZhc2NlbmQ=)

 **I. Source Code**

1. For online build and run, please use C++ or Python and comply with part 4 of programming specifications. (Currently, only C++ samples are available.)

2. For C++ PR sample specifications, see [sample](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture).

3. Organization of the project directory:   
    
    - The project directory has a **src** folder for storing the source code.   
    - The project directory has a readme file (in .md format).   
    - The project directory has a **scripts** folder which contains the **testcase*.sh** entry script of your project.
    - The project directory has a **scripts** folder which contains the **host_version.conf** configuration file that describes the product form and version mapping of your project.
    
    > **Note:** Each entry script corresponds to a product form. Your project shall provide support to at least one product form. Fill the versions mapped to each product form in the **host_version.conf** file in the corresponding directory.
    >- testcase_200dk.sh: Atlas 200 DK   
    >- testcase_300.sh: Atlas 300   
   
    > **Note:** Each product form maps to at least one version. Your project shall provide support to at least one version. The following provides a sample **host_version.conf** file.
    >- Atlas300 = c75,c73        The project adapts to C73 and C75 of Atlas 300.
    >- Atlas200dk = c73,c75        The project adapts to C73 and C75 of Atlas 200 DK.


4. For code migrated from other open-source software, add the license declaration.

 **II. License Rules**

The source code files (.cpp, .py, and .hpp files) must support the Apache License 2.0. Add a statement at the beginning of each source code file as follows:
```
# Copyright 2020 Huawei Technologies Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```
If the source code contains the copyright of other companies, add a line of Huawei's copyright as follows:
```
# Copyright 2020 Huawei Technologies Co., Ltd.
# Copyright 2018 HiSillion Technologies Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

 **III. Readme**

The readme file provides guidance for you to understand and deploy your sample. It must contain the following content:

1. Overview

    Describes the function, operation methods, and model source of the project.

2. Key requirements:

    (1) Project function, including the input and output of the sample.   
    (2) Environment variables, third-party software dependencies, and libraries, and installation methods.   
    (3) Project file download methods, by using a compressed package or the **git clone** command.   
    (4) Model download method from [ModelZoo](https://github.com/Ascend/modelzoo).  
    (5) Input requirements for offline model conversion.   
    (6) Test data (bin files, images, and videos) download method (OBS or web disk links).   
    (7) Command for converting an original model to a Da Vinci model.   
    (8) Project build and run procedure.  
    (9) Post-inference validation method (by using a shell script or Python script).   

 **IV. Regulations on PR Submission**

For details about how to submit a PR, see [How Do I Fork a Repository and Submit a PR?](https://github.com/Ascend/samples/wikis/%E5%A6%82%E4%BD%95fork%E4%BB%93%E5%BA%93%E5%B9%B6%E6%8F%90%E4%BA%A4PR?sort_id=3271318).

A submitted PR sample must contain the required items, project test cases, and a readme file.

1. Overview

   - Required items: 

        1. The project directory has a **src** folder for storing the source code.   
        2. The project directory has a readme file (in .md format).   
        3. The project directory has a **scripts** folder which contains the **testcase*.sh** entry script of your project.
        4. The project directory has a **scripts** folder which contains the **host_version.conf** configuration file.   

   - After the PR is submitted, the pipe of checking the required items is triggered, which build your project by using the shell entry script (**scripts/testcase*.sh** in the project directory). Note the following points:

        1. The PR shall not contain the **rm** command.   
        2. The PR shall not upload the original model and converted Da Vinci model.   
        3. The PR shall not upload the test dataset and validation dataset.   

2. The **scripts/testcase*.sh** script must provide the following functions:   
    Inference and post-inference validation. 
  
    The inference phase shall include the following actions:   
    - Download the test dataset and validation dataset.    
    - Download the original model.   
    - Set the environment variables required for model conversion.   
    - Run the **atc** command to convert the model.   
    - Set the environment variables required for building the project.   
    - Set the environment variables required for running the application.  
    - Run the application.    

    The post-inference validation method can be provided in the script. The validation phase shall consider the following validation results:   
    - If an error occurs in the inference phase, **inferenceError** is returned.     
    - If an error occurs in the validation phase, **verifyResError** is returned.    
    - If both phases are successful, a success message is returned.   

3. The necessary components and Python 3.6.9 environment have been pre-installed in the operating environment. Python 3.6.9 can be installed using the **python3.6** command. The third-party library dependencies can be installed using either **pip3 install package --user** or **python3.6 -m pip install package --user**.

4. The model has been uploaded to the ModelZoo according to the [Contribution guide](https://github.com/Ascend/modelzoo) repository. Provide OBS or web disk links for downloading the validation files (binaries, images, and videos). Alternatively, contact the administrator to store the files to a fixed OBS address. Do not store the files in the samples repository.

5. For environment and other problems, please submit an issue.

6. For details about how to submit test cases, see the [PR Submission Sample](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture).


 **V. Programming Specifications**

- Specifications and Standards

    1. C++ programming complies with the [Google C++ Coding Guidelines](http://google.github.io/styleguide/cppguide.html). Unit testing (UT) complies with the [Googletest Primer](https://github.com/google/googletest/blob/master/googletest/docs/primer.md).  

    2. Python programming complies with the [Python PEP 8 Coding Style](https://pep8.org/). Unit testing (UT) complies with the [pytest framework](http://www.pytest.org/en/latest/).

- Remarks

    1. Use the string data type preferentially. Avoid the use of the char* data type.   
    2. Do not use **printf**. Instead, use **cout**.   
    3. Use smart pointers for memory management.   
    4. Do not call **exit** in functions.   
    5. Do not use tools such as an IDE to automatically generate code.   
    6. Minimize third-party library dependencies. If a third-party dependency is imported, the installation and usage description must be provided.   
    7. Use English comments with a comment rate within 30%–40%. Self-commenting is encouraged.   
    8. Provide comments in the function header to describe the function purpose, input parameters, and output parameters.   
    9. Provide unified error codes to facilitate error locating.   
    10. Avoid printing error-level logs that have no impact on the system.   
    11. Remove commented useless code lines. The purpose of comments shall not be removing useless code lines.
