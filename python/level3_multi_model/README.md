English|[中文](README_CN.md)

# Level 3-Industry Examples

#### Directory structure and description

This catalog is industry teaching samples. Each folder corresponds to different types of samples for users' reference. The catalog structure and specific instructions are as follows.  

| samples  | description  |
|---|---|
| [Robot_Play_Chess](./Robot_Play_Chess)  | Use the camera to capture the game chessboard situation, use the Atlas200DK to run the VGG16 model for reasoning, run the game engine on the main control platform to calculate the AI movement, control the robotic arm to play chess, and the user uses the webserver to play against the robotic arm based on the UI on the mobile terminal  |
| [Robotic_Arm_Object_Following](./Robotic_Arm_Object_Following)  |  Use Atlas200DK to run the Yolov3 model, infer the RGB data stream given by the binocular depth camera, and detect the position of the target in the image in real time. And combined with the depth data stream of the camera to control the posture of the robotic arm so that the robotic arm moves with the target |
| [mask_rcnn_image_inpainting](./mask_rcnn_image_inpainting)  | Use MaskRcnn and ImageInpainting models to specify foreground targets for input images to remove inference. Sample input: the picture of the target to be removed. Sample output: the generated mask picture of the target to be removed, the instance segmentation picture, and the picture after the target is removed  |