import numpy as np

from acl_dvpp import Dvpp
from atlas_utils.presenteragent import presenter_datatype

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6


class VggSsd(object):
    def __init__(self, acl_resource, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        #使用dvpp处理图像,当使用opencv或者PIL时则不需要创建dvpp实例
        self._dvpp = Dvpp(acl_resource)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("Release yolov3 resource finished")


    def pre_process(self, image):
        #使用dvpp将图像缩放到模型要求大小
        resized_image = self._dvpp.resize(image, self._model_width,
                                          self._model_height)
        if resized_image == None:
            print("Resize image failed")
            return None
        #输出缩放后的图像和图像信息作为推理输入数据
        return [resized_image,]

    def post_process(self, infer_output, origin_img):
        #解析推理输出数据
        detection_result_list = self._analyze_inference_output(infer_output, 
                                                               origin_img)
        #将yuv图像转换为jpeg图像
        jpeg_image = self._dvpp.jpege(origin_img)
        return jpeg_image, detection_result_list

    def _analyze_inference_output(self, infer_output, origin_img):
        #vgg ssd有两个输出,第一个输出infer_output[0]为检测到的物体个数,shape为(1,8)
        box_num = int(infer_output[0][0, 0])
        
        #第二个输出infer_output[1]为检测到的物体信息,shape为(1, 200, 8)
        box_info = infer_output[1][0]       

        detection_result_list = []
        for i in range(box_num):
            #检测到的物体置信度
            score = box_info[i, SCORE]
            if score < 0.9:
                continue 
            detection_item = presenter_datatype.ObjectDetectionResult()            
            detection_item.confidence = score
            #人脸位置框坐标, 是归一化的坐标，需要乘以图片宽高转换为图片上的坐标
            detection_item.box.lt.x = int(box_info[i, TOP_LEFT_X] * origin_img.width)
            detection_item.box.lt.y = int(box_info[i, TOP_LEFT_Y] * origin_img.height)
            detection_item.box.rb.x = int(box_info[i, BOTTOM_RIGHT_X] * origin_img.width)
            detection_item.box.rb.y = int(box_info[i, BOTTOM_RIGHT_Y] * origin_img.height)
            #将置信度组织为字符串
            detection_item.result_text = str(round(detection_item.confidence * 100, 2)) + "%"
            detection_result_list.append(detection_item)
        return detection_result_list