"""yolov3"""
import numpy as np
import sys
sys.path.append("../../../../common")
sys.path.append("../")
from acllite_imageproc import AclLiteImageProc
from presenteragent import presenter_datatype

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6
labels = ["person",
        "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
        "pizza", "donut", "cake", "chair", "sofa", "potted plant", "bed", "dining table",
        "toilet", "TV monitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]

class Yolov3(object):
    """yolov3"""
    def __init__(self, acl_resource, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = AclLiteImageProc(acl_resource)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("Release yolov3 resource finished")

    def pre_process(self, image):
        """Use dvpp to scale the image to the required size of the model"""
        resized_image = self._dvpp.crop_and_paste(image, image.width, 
                        image.height, self._model_width, self._model_height)
        if resized_image is None:
            print("Resize image failed")
            return None
        #Output the scaled image and image information as inference input data
        return resized_image

    def post_process(self, infer_output, origin_img):
        """Analyze inference output data"""
        detection_result_list = self._analyze_inference_output(infer_output, 
                                                               origin_img)
        #Convert yuv image to jpeg image
        jpeg_image = self._dvpp.jpege(origin_img)
        return jpeg_image, detection_result_list

    def _analyze_inference_output(self, infer_output, origin_img):
        """post"""
        print("infer output shape is : ", infer_output[1].shape)
        box_num = int(infer_output[1][0, 0])
        print("box num = ", box_num)
        box_num = infer_output[1][0, 0]
        box_info = infer_output[0].flatten()
        scalex = origin_img.width / self._model_width
        scaley = origin_img.height / self._model_height
        if scalex > scaley:
            scaley =  scalex
        detection_result_list = []
        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            label = labels[ids]
            score = box_info[4 * int(box_num)+n]
            detection_item = presenter_datatype.ObjectDetectionResult()
            detection_item.confidence = score
            detection_item.box.lt.x = int(box_info[0 * int(box_num)+n] * scaley)
            detection_item.box.lt.y = int(box_info[1 * int(box_num)+n] * scaley)
            detection_item.box.rb.x = int(box_info[2 * int(box_num) + n] * scaley)
            detection_item.box.rb.y = int(box_info[3 * int(box_num) + n] * scaley)
            #Organize the confidence into a string
            detection_item.result_text = label + str(round(detection_item.confidence * 100, 2)) + "%"
            detection_result_list.append(detection_item)
        return detection_result_list
