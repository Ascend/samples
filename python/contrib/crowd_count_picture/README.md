English|[中文](README_CN.md)
  
**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## crowd_count_picture Sample   
Function: Count people using the count_person.caffe model. 
Input: crowd image. 
Output: image with the number of people marked.  

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item      | Requirement                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                     | Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware  | Atlas 200 DK/Atlas 200I DK A2/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))| Currently, the Atlas 200 DK 、Atlas 200I DK A2 and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| OpenCV and Python-acllite                                      | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://github.com/Ascend/samples/tree/master/python/environment).|

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **Note: To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```
    - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**  
       ``` 
        # 1. Click Clone or Download in the upper right corner of the samples repository and click Download ZIP.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, ${HOME}/ascend-samples-master.zip.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. Obtain the source network model required by the application.
    | **Model**      | **Description**                 | **How to Obtain**                                            |
    | ------------------ | ----------------------------- | ------------------------------------------------------------ |
    | count_person.caffe | Crowd counting based on Caffe.| Download the model and .cfg files by referring to the links in **README.md** in the [ATC_count_person_caffe_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/crowdCount/ATC_count_person_caffe_AE) directory.|
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd ${HOME}/samples/python/contrib/crowd_count_picture/model    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/crowdCount/count_person.caffe.caffemodel
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/crowdCount/count_person.caffe.prototxt
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/crowdCount/insert_op.cfg
    atc --input_shape="blob1:1,3,800,1408" --weight="count_person.caffe.caffemodel" --input_format=NCHW --output="count_person.caffe" --soc_version=Ascend310 --insert_op_conf=insert_op.cfg --framework=0 --model="count_person.caffe.prototxt" 
    **PS:If the chip model is 310B, please change the config --soc_version=Ascend310 to --soc_version=Ascend310B1**
    ```

3. Obtain the test images required by the sample.
    ```
    Run the following commands to go to the data folder of the sample and download the corresponding test images:
    cd $HOME/samples/python/contrib/crowd_count_picture/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/crowdCount/crowd.jpg
    cd ../src
    ```

### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.** 
 
**Note: If the chip version is 310B1, please change some code in main.py.**
   Open the file main.py first.Then change the function 'pre_process' to :
   ```
       def pre_process(self, image):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        return yuv_image
   ```
   Then change the function 'main()' to :
   ```
   def main():    
    """
    main
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    acl_resource = AclLiteResource()
    acl_resource.init()

    crowdcount = CrowdCount(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)   
    ret = crowdcount.init()
    
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))
        
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in constants.IMG_EXT]

    for image_file in images_list:
        ori_image = cv2.imread(image_file, 1)
        ori_image = cv2.resize(ori_image,(MODEL_WIDTH,MODEL_HEIGHT))
        save_name,ext = os.path.splitext(image_file)
        cv2.imwrite(save_name+'resize'+ext, ori_image)
        image = AclLiteImage(save_name+'resize'+ext)            
        crop_and_paste_image = crowdcount.pre_process(image)
        print("pre process end")
        result = crowdcount.inference([crop_and_paste_image])              
        result_img_encode = crowdcount.post_process(crop_and_paste_image, result, image_file)
        os.remove(save_name+'resize'+ext)

    return result_img_encode
   ```
1. Run the following commands to upload the **crowd_count_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, xxx.xxx.xxx.xxx is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r $HOME/samples/python/contrib/crowd_count_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/crowd_count_picture/src    
    ```

2. <a name="step_2"></a>Run the sample.
   ```
   python3.6 main.py ../data/
   ```

### Result Viewing

After the execution is complete, an inferred image is generated in the **out** directory of the sample project.

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
