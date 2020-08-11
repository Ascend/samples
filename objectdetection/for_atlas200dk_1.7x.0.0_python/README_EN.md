English|[中文](README_EN.md)

# Image Object Detection Network Application (Python)

This application can run on the Atlas 200 DK to detect objects in images using the YOLOv3 network and output the result on the Terminal window.

## Software Preparation

Before running this sample, obtain the source code package.

1. <a name="zh-cn_topic_0228757084_section8534138124114"></a>Obtain the source code package.   
   **mkdir -p $HOME/AscendProjects**
   
   **cd $HOME/AscendProjects**
   
   **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/objectdetection_python.zip --no-check-certificate**
   
   **unzip objectdetection_python.zip**
   
   > ![](public_sys-resources/icon-note.gif) NOTE
   > - If the download using **wget** fails, run the following command to download the code:   
   **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/objectdetection_python.zip**
   > - If the download using **curl** also fails, open the download link in a browser to download the code and manually upload it to the server.

2. <a name="zh-cn_topic_0219108795_li2074865610364"></a>Obtain the original model required by the application.
   
   - Download the original model file and weight file to any directory on the Ubuntu server, for example, **$HOME/yolov3_yuv**.
     
     **mkdir -p $HOME/yolov3_yuv**
     
     **wget -P $HOME/yolov3_yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.caffemodel**
     
     **wget -P $HOME/yolov3_yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.prototxt**
     
     > ![](public_sys-resources/icon-note.gif) NOTE
     > - Original YOLOv3 network: https://github.com/maxuehao/YOLOV3/blob/master/yolov3_res18.prototxt
     > - To obtain the LICENSE of the original YOLOv3 network, visit the following website: https://github.com/maxuehao/caffe/blob/master/LICENSE
     > - The C7x version requires modification on the .prototxt file. Modify the file by referring to the following website: https://support.huaweicloud.com/usermanual-mindstudioc73/atlasmindstudio_02_0112.html
	 The modification has been completed herein. You can directly run the preceding command to download the file.

3. Convert the original model to an offline model adapted to the Ascend AI Processor.
   
   1. Environment variable setting
      
      Set the following environment variables:
      
      **cd \$HOME/yolov3_yuv**
      
      **export install_path=\$HOME/Ascend/ascend-toolkit/20.0.RC1/x86_64-linux_gcc7.3.0** 
      
      **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH** 
      
      **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH** 
      
      **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64:\\$LD_LIBRARY_PATH**  
      
      **export ASCEND_OPP_PATH=\\${install_path}/opp**
   
   2. Convert the model.
      
      **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_yuv --soc_version=Ascend310 --insert_op_conf=aipp_nv12.cfg**

4. Upload the converted model file (.om) to the **objectdetection_python/model** directory under the path of the source code downloaded in [Step 1](#zh-cn_topic_0219108795_li953280133816).
   
   **cp ./yolov3_yuv.om \$HOME/AscendProjects/objectdetection_python/model/**

## Environment Deployment<a name="zh-cn_topic_0228757083_section1759513564117"></a>

1. Copy the application code to the developer board.
   
   Go to the root directory of the classification network application code \(Python\) as the Mind Studio installation user, for example, **AscendProjects/objectdetection_python**, and run the following command to copy the application code to the developer board:
   
   **scp -r \\$HOME/AscendProjects/objectdetection_python HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS**
   
   Enter the password of the developer board as prompted. The default password is **Mind@123**, as shown in [Figure Copying the application code.](#zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014)
   
   **Figure** Copying the application code<a name="zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014"></a>
   
   ![](figures/zh-cn_image_0228832431.png)

2. Copy the **acl.so** file to the development board.
   
   **scp ${HOME}/Ascend/ascend-toolkit/X.X.X/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl/acl.so HwHiAiUser@192.168.1.2:/home/HwHiAiUser/Ascend/**
   
   > ![](public_sys-resources/icon-note.gif) NOTE   
             **Replace  with the actual version of the Ascend Toolkit.**   
			 
			 
             **For example, if the Toolkit is named Ascend-Toolkit-20.0.RC1-x86_64-linux_gcc7.3.0.run, the Toolkit version is **20.0.RC1**.**

3. Log in to the developer board and add environment variables.
   
   **ssh HwHiAiUser@192.168.1.2**  
   **vim \${HOME}/.bashrc**   
   Append the following two lines:   
   **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64**   
   **export PYTHONPATH=/home/HwHiAiUser/Ascend/:\\${PYTHONPATH}**
   ![](figures/pythonpath.png)   
   Run the following command for the environment variables to take effect:   
   **source \${HOME}/.bashrc**

4. Install the environment dependencies.
   
   - Install NumPy and Pillow.   
       For details about the installation, see **https://gitee.com/ascend/common/blob/master/install_python3env/for_atlas200dk/README.md**.

## Run

1. Log in to the developer board, go to the project directory, and run the following command to run the application:
   
   **cd \${HOME}/HIAI_PROJECTS/objectdetection_python/**   
   **python3 object_detect.py ./data/**

2. View the inference result on the Terminal window.
   
   ![image-20200725185820768](figures/obj_res.png)

3. View the result images.
   
   The result images are stored in the **outputs** folder and can be uploaded to the home directory of the Mind Studio installation user.   
   **scp -r username@host\_ip:/home/username/HIAI\_PROJECTS/objectdetection_python/outputs \~**
   
   - **username**: user name of the developer board, default to **HwHiAiUser**.
   - **host\_ip**: IP address of the developer board. Generally, for USB connection, the IP address is **192.168.1.2**. For network cable connection, the IP address if **192.168.0.2**.
   
   **Example:**   
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS/objectdetection_python/outputs \~** 