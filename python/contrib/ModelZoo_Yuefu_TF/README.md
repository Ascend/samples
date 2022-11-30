English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample works with CANN 3.1.0 and later versions, and supports Atlas 800 (model: 9000) and Atlas 800 (model: 9010).**


### Yuefu Poetry Generation Sample

Function: generates a Chinese poem.

Input: poem title and poem type

Output:  a Yuefu poem

### Prerequisites

Before deploying this sample, ensure that:

- Ensure that the environment has been set up by referring to [Environment Preparation and Dependency Installation](https://gitee.com/ascend/samples/tree/master/python/environment).

- The development environment and operating environment of the corresponding product have been set up.

1. Obtain the source package.

   You can download the source code in the following way.

   In the development environment, run the following commands as the running user.

   ```
   cd $HOME
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/yuefuzuoshi/ModelZoo_Yuefu_TF.zip
   ```

   In the development environment, run the following commands to unzip the package:

   ```
   cd $HOME
   unzip ModelZoo_Yuefu_TF.zip
   ```

2. Obtain the network model required by the application.

   Go to the **models** directory extracted from the source package:

   ```
   cd $HOME/ModelZoo_Yuefu_TF/models
   ```

   Download the model files:

   ```
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/yuefuzuoshi/poetry.data-00000-of-00001
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/yuefuzuoshi/poetry.meta
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/yuefuzuoshi/poetry.index
   ```

 ### Sample Deployment
**The following assumes that the development environment and operating environment are set up on the same server.**   
- A script for running on a single device is provided in the source package. Before running the script, you need to edit the script file.

  (1) Configure **device_id** and **device_ip** in the **device_table_1p.json** file.

  Run the **cat /etc/hccn.conf** command to query the IP address list of the device on the server. If the IP address is not configured, configure it by referring to [Configuring NIC IP Address of a Device](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/softwareinstall/instg/atlasdeploy_03_0060.html).

  Change the values of **device_id** in line 16 and **device_ip** in line 17 in the **device_table_1p.json** file to the queried device ID and IP address, respectively.

  (2) Edit the **main_1p.sh** script to configure the following parameters.

  Change **DEVICE_ID** to the **device_id** value configured in the previous step.

  **max_decode_len**: sets the maximum characters of the poem. Defaults to **80**.

  **title**: sets the title of the poem.

  **type**: sets the type of the poem, selected from the five-character-regular-verse, five-character-quatrain, seven-character-regular-verse, and seven-character-quatrain.


### Sample Running

Run the following command:

```
bash main_1p.sh
```

### Result Checking

After the execution is complete, you should see a poem. The following is an example only.
```
七言绝句(格式)中秋(诗歌标题)
青山万古水千波，
此夕清光奈老何。
无限世间闲草木，
不知强有几人多。
```
