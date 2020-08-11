English|[中文](README.md)

# Image Object Classification Network Application (Python)

This application can run on the Atlas 200 DK to classify objects in images using the GoogLeNet network and output the top five classes with the highest confidence scores.

## Software Preparation

Before running this sample, obtain the source code package.

1. <a name="zh-cn_topic_0228757084_section8534138124114"></a>Obtain the source code package.   
   **mkdir -p $HOME/AscendProjects**
   
   **cd $HOME/AscendProjects**
   
   **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification_python.zip --no-check-certificate**
   
   **unzip classification_python.zip**
   
   > ![](public_sys-resources/icon-note.gif) NOTE
   > 
   > - If the download using **wget** fails, run the following command to download the code:   
   **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification_python.zip**
   > - If the download using **curl** also fails, open the download link in a browser to download the code and manually upload it to the server.

2. <a name="zh-cn_topic_0219108795_li2074865610364"></a>Obtain the original model required by the application.
   
   Obtain the original model file and weight file by referring to [Table 1 Model for an image classification application](#zh-cn_topic_0219108795_table19942111763710) and store them to any directory of the Ubuntu server where Mind Studio is installed.
   
   **Table 1** Model for an image classification application

    <a name="zh-cn_topic_0219108795_table19942111763710"></a>
    
    <table><thead align="left"><tr id="zh-cn_topic_0219108795_row611318123710"><th class="cellrowborder" valign="top" width="11.959999999999999%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0219108795_p81141820376"><a name="zh-cn_topic_0219108795_p81141820376"></a><a name="zh-cn_topic_0219108795_p81141820376"></a>Model name</p>
    </th>
    <th class="cellrowborder" valign="top" width="8.07%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0219108795_p13181823711"><a name="zh-cn_topic_0219108795_p13181823711"></a><a name="zh-cn_topic_0219108795_p13181823711"></a>Model description</p>
    </th>
    <th class="cellrowborder" valign="top" width="79.97%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0219108795_p1717182378"><a name="zh-cn_topic_0219108795_p1717182378"></a><a name="zh-cn_topic_0219108795_p1717182378"></a>Model download link</p>
    </th>
    </tr>
    </thead>
    <tbody><tr id="zh-cn_topic_0219108795_row1119187377"><td class="cellrowborder" valign="top" width="11.959999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0219108795_p4745165253920"><a name="zh-cn_topic_0219108795_p4745165253920"></a><a name="zh-cn_topic_0219108795_p4745165253920"></a>googlenet</p>
    </td>
    <td class="cellrowborder" valign="top" width="8.07%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0219108795_p1874515218391"><a name="zh-cn_topic_0219108795_p1874515218391"></a><a name="zh-cn_topic_0219108795_p1874515218391"></a>Image classification model
    It is a GoogLeNet model based on Caffe.</p>
    
    </td>
    <td class="cellrowborder" valign="top" width="79.97%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0219108795_p611318163718"><a name="zh-cn_topic_0219108795_p611318163718"></a><a name="zh-cn_topic_0219108795_p611318163718"></a>Download the original model file and weight file by referring to README_en.md at <a href="https://gitee.com/HuaweiAscend/models/blob/master/computer_vision/classification/googlenet/README_en.md" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/blob/master/computer_vision/classification/googlenet/README_en.md</a>. </p>
    </td>
    </tr>
    </tbody>
    </table>
3. Convert the original model to an offline model adapted to the Ascend AI Processor.
   
   1. Choose **Tools** \> **Model **Converter**** from the menu bar of Mind Studio.
   2. Configure the model conversion settings in the displayed **Model Converter** dialog box.
   3. Set the parameters by referring to the following figure.
      - Select the model file downloaded in [Step 2](#zh-cn_topic_0219108795_li2074865610364) in **Model File**, and **Weight File** is automatically filled in.
      - Set the model name to **googlenet_yuv**. To change the model name, modify the value of **MODEL_PATH** in the **classify.py** file.
      - Set **Input Type** to **FP32** and set the value of **N** in **Input Node: data** to **1**.
      - Set **Image Format** to **BGR**. In this case, inputs of the model need to be in BGR format. ![](figures/模型转换1.png "Model conversion 1")  
    ![](figures/模型转换2.png "Model conversion 2")  
    ![](figures/模型转换3.png "Model conversion 3")

4. Save the converted model to the **model** directory in the project file.   
    **cp ~/modelzoo/googlenet_yuv/device/googlenet_yuv.om ~/AscendProjects/classification_python/model/**

## Environment Deployment<a name="zh-cn_topic_0228757083_section1759513564117"></a>

1. Copy the application code to the developer board.
   
   Go to the root directory of the classification network application code \(Python\) as the Mind Studio installation user, for example, **AscendProjects/classification_python**, and run the following command to copy the application code to the developer board:
   
   **scp -r ~/AscendProjects/classification_python HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS**
   
   Enter the password of the developer board as prompted. The default password is **Mind@123**, as shown in [Figure Copying the application code.](#zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014)
   
   **Figure** Copying the application code<a name="zh-cn_topic_02287570831_zh-cn_topic_0198304761_fig1660453512014"></a>
   
   ![](figures/zh-cn_image_02288324312.png)

2. Copy the **acl.so** file to the development board.
   
   **scp ~/Ascend/ascend-toolkit/20.0.RC1/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl/acl.so HwHiAiUser@192.168.1.2:/home/HwHiAiUser/Ascend/**
   
   > ![](public_sys-resources/icon-note.gif) NOTE   
   >- **Replace  with the actual version of the Ascend Toolkit.**   
      For example, if the Toolkit is named Ascend-Toolkit-20.0.RC1-x86_64-linux_gcc7.3.0.run, the Toolkit version is **20.0.RC1**.

3. Log in to the developer board and add environment variables.
   
   **ssh HwHiAiUser@192.168.1.2**  
   **vim ~/.bashrc**   
   Append the following two lines:   
   **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64**  
   **export PYTHONPATH=/home/HwHiAiUser/Ascend/:\\${PYTHONPATH}**  
   ![](figures/pythonpath1.png)   
   Run the following command for the environment variables to take effect:   
   **source ~/.bashrc**

4. Install the environment dependencies.
   
   - Install NumPy and Pillow.   
       For details about the installation, see **https://gitee.com/ascend/common/blob/master/install_python3env/for_atlas200dk/README.md**.

## Run

1. Log in to the developer board, go to the project directory, and run the following command to run the application:
   
   **cd ~/HIAI_PROJECTS/classification_python/**  
   **python3 classify.py ./data/**

2. View the inference result on the Terminal window.
   
   ![](figures/classification.png)

3. View the result images.
   
   The result images are stored in the **outputs** folder and can be uploaded to the home directory of the Mind Studio installation user.   
   **scp -r username@host\_ip:/home/username/HIAI\_PROJECTS/classification_python/outputs \~**
   
   - **username**: user name of the developer board, default to **HwHiAiUser**.
   - **host\_ip**: IP address of the developer board. Generally, for USB connection, the IP address is **192.168.1.2**. For network cable connection, the IP address if **192.168.0.2**.
   
   **Example:**   
   **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS/classification_python/outputs \~** 