English|[中文](README_CN.md) 
 
**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode.**

## BERT Text Classification Sample
Function: uses the BERT model to classify text.  
Input: text to be inferred for classification.   
Output: text type.   

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item| Requirement| Remarks|
|---|---|---|
| CANN version| ≥ 5.0.4| Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the About Ascend Samples Repository. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md). |
| Hardware| Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| python-acllite | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://gitee.com/ascend/samples/tree/master/python/environment).|

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```
    - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**  
       ``` 
        # 1. Click **Clone** or **Download** in the upper right corner of the samples repository and click **Download ZIP**.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, **${HOME}/ascend-samples-master.zip**.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. Obtain the source model required by the application.
    |  **Model** |  **Description** |  **How to Obtain** |
    |---|---|---|
    |  bert_text_classification | An inference model for text classification| Download the model and weight files by referring to the links in **README.md** in the [**ATC_bert_classification_tf_AE**](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/nlp/bert_text_classification/ATC_bert_classification_tf_AE) directory of the ModelZoo repository. |
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification/model     
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/bert_text_classification.pb        
    atc --model=bert_text_classification.pb --framework=3 --input_format="ND" --output=bert_text_classification --input_shape="input_1:1,300;input_2:1,300" --out_nodes=dense_1/Softmax:0 --soc_version=Ascend310 --op_select_implmode="high_precision"
    ```

3. Obtain the test images required by the sample.
    ```
    cd ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/sample.txt
    ```

### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**  

1. Run the following commands to upload the **bert_text_classification** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, *xxx.xxx.xxx.xxx* is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/bert_text_classification/src
    ```

2. <a name="step_2"></a>Run the project.
    ```
    python3.6 bert_text_classification.py
    ```
​       
### Result Viewing

After the execution is complete, the inference result is displayed in the command line in the operating environment. You can edit **text in data/sample.txt** to perform a test. Currently, the model supports the classification of five text types: sports, health, military, education, and automobile.
The result is as follows:
![Input Image Description]( https://images.gitee.com/uploads/images/2021/1110/101306_d08122bf_8083019.png "txt.png")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
