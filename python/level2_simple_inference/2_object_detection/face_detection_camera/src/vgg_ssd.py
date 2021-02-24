"""vgg_ssd"""
import numpy as np
import sys
sys.path.append("../../../../common")
sys.path.append("../")
from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.presenteragent import presenter_datatype

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6


class VggSsd(object):
    """vggssd"""
    def __init__(self, acl_resource, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        #Use dvpp to process images, when using opencv or PIL, 
        # you don't need to create a dvpp instance
        self._dvpp = Dvpp(acl_resource)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("Release yolov3 resource finished")

    def pre_process(self, image):
        """Use dvpp to scale the image to the required size of the model"""
        resized_image = self._dvpp.resize(image, self._model_width,
                                          self._model_height)
        if resized_image is None:
            print("Resize image failed")
            return None
        #Output the scaled image and image information as inference input data
        return [resized_image,]

    def post_process(self, infer_output, origin_img):
        """Analyze inference output data"""
        detection_result_list = self._analyze_inference_output(infer_output, 
                                                               origin_img)
        #Convert yuv image to jpeg image
        jpeg_image = self._dvpp.jpege(origin_img)
        return jpeg_image, detection_result_list

    def _analyze_inference_output(self, infer_output, origin_img):
        #vgg ssd has two outputs, the first output 
        # infer_output[0] is the number of detected objects, and the shape is (1,8)
        box_num = int(infer_output[0][0, 0])
        #The second output infer_output[1] is the detected object information, the shape is (1, 200, 8)
        box_info = infer_output[1][0]  
        detection_result_list = []
        for i in range(box_num):
            #Detected object confidence
            score = box_info[i, SCORE]
            if score < 0.9:
                break 
            
            detection_item = presenter_datatype.ObjectDetectionResult()            
            detection_item.confidence = score
            #Person face position frame coordinates, normalized coordinates, 
            # need to be multiplied by the width and height of the picture to convert to the coordinates on the picture
            detection_item.box.lt.x = int(box_info[i, TOP_LEFT_X] * origin_img.width)
            detection_item.box.lt.y = int(box_info[i, TOP_LEFT_Y] * origin_img.height)
            detection_item.box.rb.x = int(box_info[i, BOTTOM_RIGHT_X] * origin_img.width)
            detection_item.box.rb.y = int(box_info[i, BOTTOM_RIGHT_Y] * origin_img.height)
            #Organize the confidence into a string
            detection_item.result_text = str(round(detection_item.confidence * 100, 2)) + "%"
            detection_result_list.append(detection_item)
            
        return detection_result_list
