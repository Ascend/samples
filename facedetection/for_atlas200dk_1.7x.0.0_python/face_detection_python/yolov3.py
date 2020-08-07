import numpy as np

from acl_dvpp import Dvpp
from atlas_utils.presenteragent import presenter_datatype

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

TOP_LEFT_X = 0
TOP_LEFT_Y = 1
BOTTOM_RIGHT_X = 2
BOTTOM_RIGHT_Y = 3
SCORE = 4
LABEL = 5


class Yolov3(object):
    def __init__(self, acl_resource, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        #使用dvpp处理图像,当使用opencv或者PIL时则不需要创建dvpp实例
        self._dvpp = Dvpp(acl_resource)
        #创建yolov3网络的图像信息输入数据
        self._image_info = np.array([model_width, model_height,
                                     model_width, model_height],
                                     dtype=np.float32)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("Release yolov3 resource finished")


    def pre_process(self, image):
        #使用dvpp将图像缩放到模型要求大小
        resized_image = self._dvpp.resize(image, self._model_width,
                                          self._model_height)
        #输出缩放后的图像和图像信息作为推理输入数据
        return [resized_image, self._image_info]

    def post_process(self, infer_output, origin_img):
        #解析推理输出数据
        detection_result_list = self._analyze_inference_output(infer_output, 
                                                               origin_img)
        #将yuv图像转换为jpeg图像
        jpeg_image = self._dvpp.jpege(origin_img)
        return jpeg_image, detection_result_list

    def _analyze_inference_output(self, infer_output, origin_img):
        #yolov3网络有两个输出,第二个(下标1)输出为框的个数
        box_num = int(infer_output[1][0, 0])
        #第一个(下标0)输出为框信息
        box_info = infer_output[0]
        #输出的框信息是在mode_width*model_height大小的图片上的坐标
        #需要转换到原始图片上的坐标
        scalex = origin_img.width / self._model_width
        scaley = origin_img.height / self._model_height
        detection_result_list = []
        for i in range(box_num):
            #检测到的物体类别编号
            id = int(box_info[0, LABEL * box_num + i])
            if id >= len(labels):
                print("class id %d out of range" % (id))
                continue
            detection_item = presenter_datatype.ObjectDetectionResult()
            detection_item.object_class = id
            #检测到的物体置信度
            detection_item.confidence = box_info[0, SCORE * box_num + i]
            #物体位置框坐标
            detection_item.box.lt.x = int(box_info[0, TOP_LEFT_X * box_num + i] * scalex)
            detection_item.box.lt.y = int(box_info[0, TOP_LEFT_Y * box_num + i] * scaley)
            detection_item.box.rb.x = int(box_info[0, BOTTOM_RIGHT_X * box_num + i] * scalex)
            detection_item.box.rb.y = int(box_info[0, BOTTOM_RIGHT_Y * box_num + i] * scaley)
            #将置信度和类别名称组织为字符串
            if labels == []:
                detection_item.result_text = str(detection_item.object_class) + " " + str(
                    round(detection_item.confidence * 100, 2)) + "%"
            else:
                detection_item.result_text = str(labels[detection_item.object_class]) + " " + str(
                    round(detection_item.confidence * 100, 2)) + "%"
            detection_result_list.append(detection_item)
        return detection_result_list