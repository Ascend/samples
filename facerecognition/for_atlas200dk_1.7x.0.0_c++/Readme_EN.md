English|[中文](Readme.md)

**This case is only used for learning. It is not responsible for the effect and does not support commercial use.**

# Facial Recognition<a name="EN-US_TOPIC_0232606707"></a>

Developers can deploy the application on the Atlas 200 DK to implement face registrations, predict the face information in a video by using cameras, and compare the predicted face with the registered face to output the most possible user.

The applications in the current version branch adapt to  [DDK&RunTime](https://ascend.huawei.com/resources) **1.32.0.0 and later**.

## Prerequisites<a name="en-us_topic_0203223340_section137245294533"></a>

Before deploying this sample, ensure that:

-   Mind Studio  has been installed.
-   The Atlas 200 DK developer board has been connected to  Mind Studio, the cross compiler has been installed, the SD card has been prepared, and basic information has been configured.

## Software Preparation<a name="en-us_topic_0203223340_section8534138124114"></a>

Before running the sample, obtain the source code package and configure the environment as follows:

1.  <a name="en-us_topic_0203223340_li953280133816"></a>Obtain the source code package.
    1.  By downloading the package

        Download all the code in the repository at  [https://gitee.com/Atlas200DK/sample-facialrecognition/tree/1.3x.0.0/](https://gitee.com/Atlas200DK/sample-facialrecognition/tree/1.3x.0.0/)  to any directory on Ubuntu Server where Mind Studio is located as the Mind Studio installation user, for example,  **$HOME/AscendProjects/sample-facialrecognition**.

    2.  By running the  **git**  command

        Run the following command in the  **$HOME/AscendProjects**  directory to download code:

        **git clone https://gitee.com/Atlas200DK/sample-facialrecognition.git --branch 1.3x.0.0**

2.  <a name="en-us_topic_0203223340_li99811487013"></a>Obtain the source network model required by the application.

    Obtain the source network model and its weight file used in the application by referring to  [Table 1](#en-us_topic_0203223340_table97791025517)  and save them to the same directory on Ubuntu Server where Mind Studio is located, for example,  **$HOME/models/facialrecognition**.

    **Table  1**  Models used in facial recognition

    <a name="en-us_topic_0203223340_table97791025517"></a>
    <table><thead align="left"><tr id="en-us_topic_0203223340_row48791253115"><th class="cellrowborder" valign="top" width="19%" id="mcps1.2.4.1.1"><p id="en-us_topic_0203223340_p187902511114"><a name="en-us_topic_0203223340_p187902511114"></a><a name="en-us_topic_0203223340_p187902511114"></a>Model Name</p>
    </th>
    <th class="cellrowborder" valign="top" width="23%" id="mcps1.2.4.1.2"><p id="en-us_topic_0203223340_p148791259118"><a name="en-us_topic_0203223340_p148791259118"></a><a name="en-us_topic_0203223340_p148791259118"></a>Description</p>
    </th>
    <th class="cellrowborder" valign="top" width="57.99999999999999%" id="mcps1.2.4.1.3"><p id="en-us_topic_0203223340_p987922511111"><a name="en-us_topic_0203223340_p987922511111"></a><a name="en-us_topic_0203223340_p987922511111"></a>Download Path</p>
    </th>
    </tr>
    </thead>
    <tbody><tr id="en-us_topic_0203223340_row38791825912"><td class="cellrowborder" valign="top" width="19%" headers="mcps1.2.4.1.1 "><p id="en-us_topic_0203223340_p0879152519115"><a name="en-us_topic_0203223340_p0879152519115"></a><a name="en-us_topic_0203223340_p0879152519115"></a>face_detection</p>
    </td>
    <td class="cellrowborder" valign="top" width="23%" headers="mcps1.2.4.1.2 "><p id="en-us_topic_0203223340_p9879112516111"><a name="en-us_topic_0203223340_p9879112516111"></a><a name="en-us_topic_0203223340_p9879112516111"></a>Network model for face detection.</p>
    <p id="en-us_topic_0203223340_p1087912253112"><a name="en-us_topic_0203223340_p1087912253112"></a><a name="en-us_topic_0203223340_p1087912253112"></a>It is converted from the Caffe-based ResNet10-SSD300 model.</p>
    </td>
    <td class="cellrowborder" valign="top" width="57.99999999999999%" headers="mcps1.2.4.1.3 "><p id="en-us_topic_0203223340_p188801525813"><a name="en-us_topic_0203223340_p188801525813"></a><a name="en-us_topic_0203223340_p188801525813"></a>Download the source network model file and its weight file by referring to<strong id="en-us_topic_0203223340_b59791410131920"><a name="en-us_topic_0203223340_b59791410131920"></a><a name="en-us_topic_0203223340_b59791410131920"></a> README.md</strong> at <a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/face_detection" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/face_detection</a>.</p>
    </td>
    </tr>
    <tr id="en-us_topic_0203223340_row11880162511114"><td class="cellrowborder" valign="top" width="19%" headers="mcps1.2.4.1.1 "><p id="en-us_topic_0203223340_p1388012251117"><a name="en-us_topic_0203223340_p1388012251117"></a><a name="en-us_topic_0203223340_p1388012251117"></a>vanillacnn</p>
    </td>
    <td class="cellrowborder" valign="top" width="23%" headers="mcps1.2.4.1.2 "><p id="en-us_topic_0203223340_p1988018251110"><a name="en-us_topic_0203223340_p1988018251110"></a><a name="en-us_topic_0203223340_p1988018251110"></a>Network model for marking facial feature points.</p>
    <p id="en-us_topic_0203223340_p588013251514"><a name="en-us_topic_0203223340_p588013251514"></a><a name="en-us_topic_0203223340_p588013251514"></a>It is a network model converted from the VanillaCNN model based on Caffe.</p>
    </td>
    <td class="cellrowborder" valign="top" width="57.99999999999999%" headers="mcps1.2.4.1.3 "><p id="en-us_topic_0203223340_p28801025319"><a name="en-us_topic_0203223340_p28801025319"></a><a name="en-us_topic_0203223340_p28801025319"></a>Download the source network model file and its weight file by referring to<strong id="en-us_topic_0203223340_b1992849103910"><a name="en-us_topic_0203223340_b1992849103910"></a><a name="en-us_topic_0203223340_b1992849103910"></a> README.md</strong> at <a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/vanillacnn" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/vanillacnn</a>.</p>
    </td>
    </tr>
    <tr id="en-us_topic_0203223340_row988092511120"><td class="cellrowborder" valign="top" width="19%" headers="mcps1.2.4.1.1 "><p id="en-us_topic_0203223340_p108806251513"><a name="en-us_topic_0203223340_p108806251513"></a><a name="en-us_topic_0203223340_p108806251513"></a>sphereface</p>
    </td>
    <td class="cellrowborder" valign="top" width="23%" headers="mcps1.2.4.1.2 "><p id="en-us_topic_0203223340_p68802251019"><a name="en-us_topic_0203223340_p68802251019"></a><a name="en-us_topic_0203223340_p68802251019"></a>Network model for obtaining feature vectors.</p>
    <p id="en-us_topic_0203223340_p148801125512"><a name="en-us_topic_0203223340_p148801125512"></a><a name="en-us_topic_0203223340_p148801125512"></a>It is a network model converted from the SphereFace model based on Caffe.</p>
    </td>
    <td class="cellrowborder" valign="top" width="57.99999999999999%" headers="mcps1.2.4.1.3 "><p id="en-us_topic_0203223340_p128806251116"><a name="en-us_topic_0203223340_p128806251116"></a><a name="en-us_topic_0203223340_p128806251116"></a>Download the source network model file and its weight file by referring to<strong id="en-us_topic_0203223340_b290214112396"><a name="en-us_topic_0203223340_b290214112396"></a><a name="en-us_topic_0203223340_b290214112396"></a> README.md</strong> at <a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/sphereface" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/sphereface</a>.</p>
    </td>
    </tr>
    </tbody>
    </table>

3.  Log in to Ubuntu Server where Mind Studio is located as the Mind Studio installation user, confirm the current DDK version, and set the environment variables  **DDK\_HOME**,  **tools\_version**, and  **LD\_LIBRARY\_PATH**.
    1.  <a name="en-us_topic_0203223340_en-us_topic_0203223294_li61417158198"></a>Query the current DDK version.

        A DDK version can be queried by using either Mind Studio or the DDK software package.

        -   Using Mind Studio

            On the project page of Mind Studio, choose  **File \> Settings \> System Settings \> Ascend DDK**  to query the DDK version.

            **Figure  1**  Querying the DDK version<a name="en-us_topic_0203223340_en-us_topic_0203223294_fig17553193319118"></a>  
            ![](figures/querying-the-ddk-version.png "querying-the-ddk-version")

            The displayed  **DDK Version**  is the current DDK version, for example,  **1.32.0.B080**.

        -   Using the DDK software package

            Obtain the DDK version based on the DDK package name.

            DDK package name format:  **Ascend\_DDK-\{software version\}-\{interface version\}-x86\_64.ubuntu16.04.tar.gz**

            _Software version_  indicates the DDK software version.

            For example:

            If the DDK package name is  **Ascend\_DDK-1.32.0.B080-1.1.1-x86\_64.ubuntu16.04.tar.gz**, the DDK version is  **1.32.0.B080**.

    2.  Set environment variables.

        **vim \~/.bashrc**

        Run the following commands to add the environment variables  **DDK\_HOME**  and  **LD\_LIBRARY\_PATH**  to the last line:

        **export tools\_version=_1.32.X.X_**

        **export DDK\_HOME=$HOME/.mindstudio/huawei/ddk/_1.32.X.X_/ddk**

        **export LD\_LIBRARY\_PATH=$DDK\_HOME/lib/x86\_64-linux-gcc5.4**

        >![](public_sys-resources/icon-note.gif) **NOTE:**   
        >-   **_1.32.X.X_**  indicates the DDK version queried in  [a](#en-us_topic_0203223340_en-us_topic_0203223294_li61417158198). Set this parameter based on the query result, for example,  **1.32.0.B080**.  
        >-   If the environment variables have been added, skip this step.  

        Type  **:wq!**  to save settings and exit.

        Run the following command for the environment variable to take effect:

        **source \~/.bashrc**

4.  Convert the source network model to a model supported by the Ascend AI processor. A model can be converted either using Mind Studio or in CLI mode.
    -   Convert a model using Mind Studio.
        1.  Choose  **Tools \> Model Convert**  from the main menu of Mind Studio.
        2.  On the  **Model Conversion**  page that is displayed, configure model conversion.
            -   Select the model file downloaded in  [Step 2](#en-us_topic_0203223340_li99811487013)  for  **Model File**. The weight file is automatically matched and filled in  **Weight File**.
            -   Set  **Model Name**  to the model name in  [Table 1](#en-us_topic_0203223340_table97791025517).
            -   Set the following parameters for vanilla CNN model conversion:

                -   In the  **Nodes**  configuration, change the value of  **N**  in  **Input Node:data**  to  **4**. The value of this parameter must be the same as that of  **batch\_size**  of the corresponding model in  **graph\_template.config**. Retain the default values of  **C**,  **H**, and  **W**, as shown in  [Figure 2](#en-us_topic_0203223340_fig5158834193915).
                -   In the  **AIPP**  configuration, set  **Image Preprocess**  to  **off**.

                **Figure  2**  Nodes configuration for vanilla CNN model conversion<a name="en-us_topic_0203223340_fig5158834193915"></a>  
                

                ![](figures/model_facial_1.png)

            -   Set the following parameters for SphereFace model conversion:

                -   In the  **Nodes**  configuration,  **N:8**  in  **Input Node:data**  indicates that 8 images are processed each time for the facial recognition application. The value of this parameter must be the same as the value of  **batch\_size**  of the corresponding model in  **graph.config**.
                -   In the  **AIPP**  configuration, set **Input Image Format** to  **RGB888\_U8**.
                -   In the  **AIPP**  configuration, set  **Input Image Size**  to  **96**  and  **112**. In this example, 128 x 16 alignment is not required.
                -   In the  **AIPP**  configuration, set **Model Image Format** to  **BGR888\_U8**.
                -   In the  **AIPP**  configuration, set  **Mean Less**  to the mean value of the images used in the model training. The value can be obtained from the  **sphereface\_model.prototxt**  file of the model.
                -   In the  **AIPP**  configuration, set  **Multiplying Factor**  to the multiplication factor of the images used in the model training. The value can be obtained from the** sphereface\_model.prototxt**  file of the model, that is, the value of  **scale**.

                **Figure  3**  Nodes configuration for SphereFace model conversion<a name="en-us_topic_0203223340_fig188415461909"></a>  
                

                ![](figures/model_facial_3.png)

                **Figure  4**  Modify the  **AIPP** configuration for SpherefaceModel conversion as shown in the following figure.<a name="en-us_topic_0203223340_fig159362210546"></a>  
                

                ![](figures/model_facial_4.png)

            -   In the face\_detection model, change the values of  **Input Image Size**  to  **384**  and  **304**  respectively. The values must be 128 x 16 aligned. Select  **BGR888\_U8**  for  **Model Image Format**. Retain the default values for other parameters.

                **Figure  5**  Parameter settings for face\_detection model conversion<a name="en-us_topic_0203223340_fig525743174114"></a>  
                

                ![](figures/model_facial_5.png)

        3.  Click  **OK**  to start model conversion.

            During the conversion of the face\_detection model, an error shown in  [Figure 6](#en-us_topic_0203223340_fig19683520164211)  is displayed.

            **Figure  6**  Model conversion error<a name="en-us_topic_0203223340_fig19683520164211"></a>  
            

            ![](figures/model_facial_conversionfailed.png)

            Select  **SSDDetectionOutput**  from the  **Suggestion**  drop-down list box at the  **DetectionOutput**  layer and click  **Retry**.

            After successful conversion, an .om offline model is generated in the  **$HOME/modelzoo/XXX/device**  directory.

            >![](public_sys-resources/icon-note.gif) **NOTE:**   
            >-   For details about the descriptions of each step and parameters in model conversion on Mind Studio, see "Model Conversion" in the  [Mind Studio User Guide](https://ascend.huawei.com/doc/mindstudio/).  
            >-   **XXX**  indicates the name of the model to be converted. For example,  **face\_detection.om**  is stored in  **$HOME/modelzoo/face\_detection/device**.  


    -   Convert a model in CLI mode.
        1.  Go to the folder for storing original models as the Mind Studio installation user.

            **cd $HOME/ascend/models/facialrecognition**

        2.  Run the following command to convert the model using OMG:

            ```
            ${DDK_HOME}/uihost/bin/omg --output="./XXX" --model="./XXX.prototxt" --framework=0 --ddk_version=${tools_version} --weight="./XXX.caffemodel" --input_shape=`head -1 $HOME/AscendProjects/sample-facialrecognition/script/shape_XXX` --insert_op_conf=$HOME/AscendProjects/sample-facialrecognition/script/aipp_XXX.cfg --op_name_map=$HOME/AscendProjects/sample-facialrecognition/script/reassign_operators
            ```

            >![](public_sys-resources/icon-note.gif) **NOTE:**   
            >-   The files required by  **input\_shape**,  **insert\_op\_conf**, and  **op\_name\_map**  are stored in the  **sample-facialrecognition/script**  directory under the source code path. Configure the file paths based on the actual source code path.  
            >-   **XXX**  indicates the name of the model used in  [Table 1](#en-us_topic_0203223340_table97791025517). Replace it with the name of the model to be converted.  
            >-   Parameters  **insert\_op\_conf**  and  **op\_name\_map**  are not required during vanilla CNN model conversion, and the  **op\_name\_map**  parameter is not required during SphereFace model conversion. If unnecessary parameters are not deleted, an error is reported during model conversion.  
            >-   For details about parameter descriptions, see "Model Conversion" in the  [Atlas 200 DK User Guide](https://ascend.huawei.com/doc/atlas200dk/).  


5.  Upload the converted .om model file to the  **sample-facialrecognition/script**  directory in the source code path in  [Step 1](#en-us_topic_0203223340_li953280133816).

## Building a Project<a name="en-us_topic_0203223340_section147911829155918"></a>

1.  Open the project.

    Go to the directory that stores the decompressed installation package as the Mind Studio installation user in CLI mode, for example,  **$HOME/MindStudio-ubuntu/bin**. Run the following command to start Mind Studio:

    **./MindStudio.sh**

    After the startup is successful, open the  **sample-facialrecognition**  project, as shown in  [Figure 7](#en-us_topic_0203223340_fig28591855104218).

    **Figure  7**  Opening the sample-facialrecognition project<a name="en-us_topic_0203223340_fig28591855104218"></a>  
    ![](figures/opening-the-sample-facialrecognition-project.png "opening-the-sample-facialrecognition-project")

2.  Configure project information in the  **src/param\_configure.conf**  file.

    For details, see  [Figure 8](#en-us_topic_0203223340_fig1338571124515).

    **Figure  8**  Configuration file path<a name="en-us_topic_0203223340_fig1338571124515"></a>  
    

    ![](figures/facial_open_src.png)

    The default configurations of the configuration file are as follows:

    ```
    remote_host=192.168.1.2
    data_source=Channel-1
    presenter_view_app_name=video
    ```

    -   **remote\_host**: IP address of the Atlas 200 DK developer board
    -   **data\_source**: camera channel. The value can be  **Channel-1**  or  **Channel-2**. For details, see  **Viewing the Channel to Which a Camera Belongs**  in  [Atlas 200 DK User Guide](https://ascend.huawei.com/doc/Atlas200DK/).
    -   **presenter\_view\_app\_name**: indicates the value of  **View Name**  on the  **Presenter Server**  page, which must be unique. The value consists of 3 to 20 characters and supports only uppercase letters, lowercase letters, digits, and underscores \(\_\).

    >![](public_sys-resources/icon-note.gif) **NOTE:**   
    >-   All the three parameters must be set. Otherwise, the build fails.  
    >-   Do not use double quotation marks \(""\) during parameter settings.  
    >-   Modify the default configurations as required.  

3.  Run the  **deploy.sh**  script to adjust configuration parameters and download and compile the third-party library. Open the  **Terminal**  window of Mind Studio. By default, the home directory of the code is used. Run the  **deploy.sh**  script in the background to deploy the environment, as shown in  [Figure 9](#en-us_topic_0203223340_fig16909182592016).

    **Figure  9**  Running the deploy.sh script<a name="en-us_topic_0203223340_fig16909182592016"></a>  
    ![](figures/running-the-deploy-sh-script.png "running-the-deploy-sh-script")

    >![](public_sys-resources/icon-note.gif) **NOTE:**   
    >-   During the first deployment, if no third-party library is used, the system automatically downloads and builds the third-party library, which may take a long time. The third-party library can be directly used for the subsequent compilation.  
    >-   During deployment, select the IP address of the host that communicates with the developer board. Generally, the IP address is the IP address configured for the virtual NIC. If the IP address is in the same network segment as the IP address of the developer board, it is automatically selected for deployment. If they are not in the same network segment, you need to manually type the IP address of the host that communicates with the developer board to complete the deployment.  

4.  Start the build. Open Mind Studio and choose  **Build \> Build \> Build-Configuration**  from the main menu. The  **build**  and  **run**  folders are generated in the directory, as shown in  [Figure 10](#en-us_topic_0203223340_fig1629455494718).

    **Figure  10**  Build and file generating<a name="en-us_topic_0203223340_fig1629455494718"></a>  
    ![](figures/build-and-file-generating.png "build-and-file-generating")

    >![](public_sys-resources/icon-notice.gif) **NOTICE:**   
    >When you build a project for the first time,  **Build \> Build**  is unavailable. You need to choose  **Build \> Edit Build Configuration**  to set parameters before the build.  

5.  <a name="en-us_topic_0203223340_li1364788188"></a>Start Presenter Server.

    Open the  **Terminal**  window of Mind Studio. By default, under the code storage path, run the following command to start the Presenter Server program of the facial recognition application on the server, as shown in  [Figure 11](#en-us_topic_0203223340_fig156364995016):

    **bash run\_present\_server.sh**

    **Figure  11**  Starting Presenter Server<a name="en-us_topic_0203223340_fig156364995016"></a>  
    

    ![](figures/facial_run_1.png)

    -   When the message  **Please choose one to show the presenter in browser\(default: 127.0.0.1\):**  is displayed, type the IP address \(usually IP address for accessing Mind Studio\) used for accessing the Presenter Server service in the browser.
    -   When the message  **Please input a absolute path to storage facial recognition data:**  is displayed, type the path for storing face registration data and parsing data on Mind Studio. The  Mind Studio  user must have the read and write permissions. If the path does not exist, the script will automatically create it.

    Select the IP address used by the browser to access the Presenter Server service in  **Current environment valid ip list**  and type the path for storing facial recognition data, as shown in  [Figure 12](#en-us_topic_0203223340_fig157571218181018).

    **Figure  12**  Project deployment<a name="en-us_topic_0203223340_fig157571218181018"></a>  
    

    ![](figures/facial_run_2.png)

    [Figure 13](#en-us_topic_0203223340_fig123741843161320)  shows that the presenter\_server service has been started successfully.

    **Figure  13**  Starting the Presenter Server process<a name="en-us_topic_0203223340_fig123741843161320"></a>  
    

    ![](figures/facial_runok.png)

    Use the URL shown in the preceding figure to log in to Presenter Server \(only the Chrome browser is supported\). The IP address is that typed in  [Figure 12](#en-us_topic_0203223340_fig157571218181018)  and the default port number is  **7009**. The following figure indicates that Presenter Server has been started successfully.

    **Figure  14**  Home page<a name="en-us_topic_0203223340_fig98461795813"></a>  
    ![](figures/home-page.png "home-page")

    The following figure shows the IP address used by Presenter Server and  Mind Studio  to communicate with the Atlas 200 DK.

    **Figure  15**  IP address example<a name="en-us_topic_0203223340_fig1627210116351"></a>  
    ![](figures/ip-address-example.png "ip-address-example")

    Where:

    -   The IP address of the Atlas 200 DK developer board is 192.168.1.2 \(connected in USB mode\).
    -   The IP address used by the Presenter Server to communicate with the Atlas 200 DK is in the same network segment as the IP address of the Atlas 200 DK on the UI Host server. For example: 192.168.1.223.
    -   The following describes how to access the IP address \(such as  **10.10.0.1**\) of Presenter Server using a browser. Because Presenter Server and  Mind Studio  are deployed on the same server, you can access  Mind Studio  through the browser using the same IP address. 


## Running<a name="en-us_topic_0203223340_section1676879104"></a>

1.  Run the facial recognition application.

    On the toolbar of Mind Studio, click  **Run**  and choose  **Run \> Run 'sample-facialrecognition'**. As shown in  [Figure 16](#en-us_topic_0203223340_fig182957429910), the executable program is running on the developer board.

    **Figure  16**  Running program<a name="en-us_topic_0203223340_fig182957429910"></a>  
    

    ![](figures/facial_run3.png)

2.  Use the URL displayed upon the start of the Presenter Server service to log in to Presenter Server. For details, see  [Start Presenter Server](#en-us_topic_0203223340_li1364788188).

    [Figure 17](#en-us_topic_0203223340_fig1189774382115)  shows the Presenter Server page.

    **Figure  17**  Presenter Server page<a name="en-us_topic_0203223340_fig1189774382115"></a>  
    ![](figures/presenter-server-page.png "presenter-server-page")

    >![](public_sys-resources/icon-note.gif) **NOTE:**   
    >-   Presenter Server of the facial recognition application supports a maximum of two channels at the same time \(each  _presenter\_view\_app\_name_  corresponds to a channel\).  
    >-   Due to hardware limitations, each channel supports a maximum frame rate of 20 fps. A lower frame rate is automatically used when the network bandwidth is low.  

3.  Register a face.
    1.  Click the  **Face Library**  tab and enter a user name in the  **Username**  text box.

        **Figure  18**  Face registration page<a name="en-us_topic_0203223340_fig12445181112163"></a>  
        ![](figures/face-registration-page.png "face-registration-page")

    2.  Click  **Browse**  to upload a face image. Crop the face image based on the ratio of  **Example Photo**.

    1.  Click  **Submit**. If the upload fails, you can change the cropping ratio.

4.  Perform facial recognition and comparison.

    On the  **App List**  tab page, click  _video_  for example in the  **App Name**  column. If a face is displayed in the camera and matches the registered face, the name and similarity information of the person are displayed.


## Follow-up Operations<a name="en-us_topic_0203223340_section1092612277429"></a>

-   **Stopping the Facial Recognition Application**

    The facial recognition application is running continually after being executed. To stop it, perform the following operation:

    Click the stop button shown in  [Figure 19](#en-us_topic_0203223340_fig12461162791610)  to stop the facial recognition application.

    **Figure  19**  Stop of the facial recognition application<a name="en-us_topic_0203223340_fig12461162791610"></a>  
    

    ![](figures/facial_stopping.png)

    [Figure 20](#en-us_topic_0203223340_fig5786125319165)  shows that the application has been stopped.

    **Figure  20**  Stop of the facial recognition application<a name="en-us_topic_0203223340_fig5786125319165"></a>  
    

    ![](figures/facial_stopped.png)

-   **Stopping the Presenter Server Service**

    The Presenter Server service is always in running state after being started. To stop the Presenter Server service of the facial recognition application, perform the following operations:

    Run the following command to check the process of the Presenter Server service corresponding to the facial recognition application as the  Mind Studio  installation user:

    **ps -ef | grep presenter | grep facial\_recognition**

    ```
    ascend@ascend-HP-ProDesk-600-G4-PCI-MT:~/sample-facialrecognition$ ps -ef | grep presenter | grep facial_recognition
    ascend 22294 20313 22 14:45 pts/24?? 00:00:01 python3 presenterserver/presenter_server.py --app facial_recognition
    ```

    In the preceding information,  _22294_  indicates the process ID of the Presenter Server service corresponding to the facial recognition application.

    To stop the service, run the following command:

    **kill -9** _22294_


