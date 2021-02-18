中文|[English](CONTRIBUTING_EN.md) 

**介绍**

Ascend Samples，欢迎各位开发者！

 **贡献要求**

开发者提交的修改的工程文件中至少包括源码、readme、编译&运行脚本，并遵循以下标准。

请贡献者在提交代码之前签署CLA协议，“个人签署”，[链接](https://clasign.osinfra.cn/sign/Z2l0ZWUlMkZhc2NlbmQ=)。

 **一、源码**

1. 在线编译,推理请使用C++/python代码实现，符合第四部分编码规范(当前只有C++的样例)

2. C++PR 提交样例规范请参考[sample](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture)。

3. 贡献者工程代码目录规则：   
    
    -   工程目录下有 src 文件夹，用于存放源码。   
    -   工程目录下有 readme (*.md 格式的文件)。   
    -   工程目录下有 scripts 文件夹, 文件夹下有当前待提交工程的入口脚本 : testcase*.sh 。
    -   工程目录下有 scripts 文件夹, 文件夹下有描述当前工程所适配的设备形态以及相应版本的配置说明文件 host_version.conf 。
    
    > **说明：** 每一个入口脚本对应一种设备形态，要求当前工程至少适配一种设备形态，每种设备的入口脚本对应的版本号须填写在同级目录下的 host_version.conf 文件中。
    >- testcase_200dk.sh   适配设备Atlas200dk。   
    >- testcase_300.sh     适配设备Atlas300。   
    >- testcase_800.sh     适配设备Atlas800。   
    >- testcase_1951.sh    适配设备1951。
   
    > **说明：** 每一个设备形态对应一种或多种版本，要求当前工程至少适配一种版本。例如 host_version.conf中内容为：
    >- Atlas300 = c75,c73         当前工程适配Atlas300的c73和c75两种版本。
    >- Atlas200dk = c73,c75       当前工程适配Atlas200dk的c73和c75两种版本。
    >- Atlas800 = c75             当前工程适配Atlas800的c75版本。
    >- Atlas1951 = c75            当前工程适配Atlas1951的c75版本。


4. 从其他开源迁移的代码，请增加License声明。

 **二、License规则**

## 如果您是华为员工：
#### 所有的新建源码文件(cpp、py、hpp文件)需要支持Apache 2.0 License，并在源码文件头部增加声明，如下所示。
```
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```
#### 所有从其他源码修改而来的代码，不要改变源代码中的LICENSE类型，如果源码中已有其它公司的Copyright，原有的copyright声明保持不变，在上面增加一行华为的Copyright，如下所示。
```
# Copyright 2021 Huawei Technologies Co., Ltd
# Copyright 2018 hisillion Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
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

## 如果您非华为员工：
#### 需要根据您签署的CLA类型进行声明：
#### CLA签署网站：https://clasign.osinfra.cn/sign/Z2l0ZWUlMkZhc2NlbmQ=
#### CLA包括企业签署、员工签署、个人签署三类，非华为员工签署个人，并根据自己签署的类型声明对应的copyright
#### 所有的新建源码文件(cpp、py、h等文件)需要支持Apache 2.0 License，并在源码文件头部增加如下声明，将[yyyy]替换为代码创建的4位数年份，将[name of the copyright owner]替换为所在组织的名字（所有个人签署CLA一律声明Huawei Technologies Co., Ltd），注意删除方括号：
```
# Copyright [yyyy] [name of copyright owner]
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
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

#### 所有从其他源码修改而来的代码，不要改变源代码中的LICENSE类型，如果源码中已有其它公司的Copyright，原有的copyright声明保持不变，在上面增加一行Copyright，将[yyyy]替换为修改代码的4位数年份，将[name of the copyright owner]替换为本人所在组织的名字（所有个人签署CLA一律声明Huawei Technologies Co., Ltd），注意删除方括号。
```
# Copyright [yyyy] [name of copyright owner]
# Copyright 2018 hisillion Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
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

 **三、readme**

readme用于指导用户理解和部署样例，要包含如下内容：

1. 简介：

    工程项目的功能和运行方法、模型的来源；

2. 关键要求：

    1. 提交的工程项目的功能：包含样例的输入和输出;   
    2. 环境变量设置，依赖的第三方软件包和库，以及安装方法；   
    3. 工程文件获取方法：下载工程文件压缩包或是git clone；   
    4. 模型如何从(modelzoo)[https://gitee.com/ascend/modelzoo]仓下载；  
    5. 模型转换得到的离线模型对输入数据的要求；   
    6. 测试数据(bin文件，图片，视频)请提供归档OBS、网盘链接;   
    7. 将原始模型转换为Davinci模型的模型转换命令；   
    8. 工程编译步骤以及工程运行步骤；  
    9. 推理后的校验方法 (校验方法以shell脚本或是python脚本提供)   

 **四、PR提交规范**

提交PR操作可以参考[如何fork仓库并提交PR_wiki](https://gitee.com/ascend/samples/wikis/%E5%A6%82%E4%BD%95fork%E4%BB%93%E5%BA%93%E5%B9%B6%E6%8F%90%E4%BA%A4PR?sort_id=3271318)。

PR提交的样例需要包含门禁项、工程测试用例和readme。

1. 简介：

   - 提交的PR中应该包含如下文件， 这是校验PR提交是否有效的门禁项： 

        1. 工程目录下有 src 文件夹，用于存放源码。   
        2. 工程目录下有 readme (*.md 格式的文件)。   
        3. 工程目录下有 scripts 文件夹, 文件夹下有testcase*.sh脚本。
        4. 工程目录下有 scripts 文件夹, 文件夹下有host_version.conf配置文件。   

   - 提交PR后，会自动触发门禁流水，后台会根据用例入口shell(工程目录下的scripts/testcase*.sh)进行编译，关键要求如下：

        1. 提交的PR中的文件不能包含 rm 删除命令；   
        2. 提交的PR中不应上传原始模型文件和转换后的Davinci模型；   
        3. 提交的PR中不应上传测试集和验证集;   

2. scripts/testcase*.sh 应该包含如下功能。   
    testcase*.sh 包含两方面功能：推理和校验推理的结果。 
  
    推理阶段包含：   
    -    下载测试集和验证集。    
    -    下载原始模型文件。   
    -    设置模型转换所需的环境变量。   
    -    使用atc命令进行模型转换。   
    -    根据自己提交的工程配置工程编译时所需的环境变量。   
    -    配置程序运行所需的环境变量。  
    -    运行程序。    

    校验推理的结果(根据不同的工程提供不同的推理结果校验方法，校验方法可以写在脚本中)：   
    -    当推理阶段发生成错误时 返回 inferenceError。     
    -    当推理结果校验阶段发生成错误时 返回 verifyResError。    
    -    如果两个阶段执行都成功 最后返回 success。   

3. 执行环境已预装软件包和Python3.6.9环境，调用命令"python3.6"、，安装第三方库依赖使用"pip3 install package --user"、"python3.6 -m pip install package --user"均可。

4. 模型需要按照 [modelzoo](https://gitee.com/ascend/modelzoo)仓的贡献指南，上传到modelzoo中。验证文件(bin、图片、视频)请提供归档OBS、网盘链接或联系管理员存放到固定的obs地址，不要放在samples仓中。

5. 环境和其他问题，请提交Issue跟踪。

6. 提交测试用例可以参考[PR提交示例](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture)。


 **五、编程规范**

- 规范标准

    1. C++代码遵循google编程规范：[Google C++ Coding Guidelines](http://google.github.io/styleguide/cppguide.html)；单元测测试遵循规范： [Googletest Primer](https://github.com/google/googletest/blob/master/googletest/docs/primer.md)  

    2. Python代码遵循PEP8规范：[Python PEP 8 Coding Style](https://pep8.org/)；单元测试遵循规范： [pytest](http://www.pytest.org/en/latest/)

- 规范备注

    1. 优先使用string类型，避免使用char*；   
    2. 禁止使用printf，一律使用cout；   
    3. 内存管理尽量使用智能指针；   
    4. 不准在函数里调用exit；   
    5. 禁止使用IDE等工具自动生成代码；   
    6. 控制第三方库依赖，如果引入第三方依赖，则需要提供第三方依赖安装和使用指导书；   
    7. 一律使用英文注释，注释率30%--40%，鼓励自注释；   
    8. 函数头必须有注释，说明函数作用，入参、出参；   
    9. 统一错误码，通过错误码可以确认那个分支返回错误；   
    10. 禁止出现打印一堆无影响的错误级别的日志；   
    11. 注释的代码如果没有特别用处，全部删掉，严禁通过注释的方式删除无用代码；
