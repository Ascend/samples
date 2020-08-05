import sys
import os
import numpy as np
import acl

from utils import *
from acl_dvpp import Dvpp
from acl_model import Model
from acl_image import AclImage
from PIL import Image, ImageDraw, ImageFont

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

class BBox(object):
    def __init__(self, ltx=0, lty=0, rbx=0, rby=0, text=None):
        self.ltx

class ObjectDetect(object):
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = None
        self._image_info_dev = None

    def __del__(self):
        if self._model:
            del self._model
        if self._dvpp:
            del self._dvpp
        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("[Sample] class Samle release source success")

    def _init_resource(self):
        print("[Sample] init resource stage:")
        ret = acl.init()
        check_ret("acl.rt.set_device", ret)

        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)

        self.run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)

        print("Init resource stage success") 

    def init(self):
        #初始化 acl 资源
        self._init_resource() 
        self._dvpp = Dvpp(self.stream, self.run_mode)

        #初始化dvpp
        ret = self._dvpp.init_resource()
        if ret != SUCCESS:
            print("Init dvpp failed")
            return FAILED
        
        #加载模型
        self._model = Model(self.run_mode, self._model_path)
        ret = self._model.init_resource()
        if ret != SUCCESS:
            print("Init model failed")
            return FAILED

        self._init_image_info()

        return SUCCESS

    def _init_image_info(self):
        image_info = np.array([self._model_width, self._model_height, 
                               self._model_width, self._model_height], 
                               dtype = np.float32) 
        ptr = acl.util.numpy_to_ptr(image_info)
        self._image_info_size = image_info.itemsize * image_info.size
        self._image_info_dev = copy_data_device_to_device(
            ptr, self._image_info_size) 

    def pre_process(self, image):
        yuv_image = self._dvpp.convert_jpeg_to_yuv(image)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        return self._model.execute(resized_image.data(), resized_image.size,
                                   self._image_info_dev, self._image_info_size)

    def post_process(self, infer_output, origin_img, image_file):
        print("post process")
        print(infer_output[1])
        box_num = infer_output[1][0, 0]
        print("box num ", box_num)
        box_info = infer_output[0].flatten()
        print(box_info[0:16])
        scalex = origin_img.width / self._model_width
        scaley = origin_img.height / self._model_height
        output_path = os.path.join("./outputs", os.path.basename(image_file))
        origin_image = Image.open(image_file)
        draw = ImageDraw.Draw(origin_image)
        font = ImageFont.truetype("SourceHanSansCN-Normal.ttf", size=30)
        print("images:{}".format(image_file))
        print("======== inference results: =============")
        for n in range(int(box_num)):
            id = int(box_info[5])
            label = labels[id]
            score = box_info[4]
            top_left_x = box_info[0] * scalex
            top_left_y = box_info[1] * scaley
            bottom_right_x = box_info[2] * scalex
            bottom_right_y = box_info[3] * scaley
            print("%s: class %d, box %d %d %d %d, score %f"%(
                label, id, top_left_x, top_left_y, 
                bottom_right_x, bottom_right_y, score))
            draw.line([(top_left_x, top_left_y),(bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), \
            (top_left_x, bottom_right_y), (top_left_x, top_left_y)],fill=(0,200,100),width=3)
            draw.text((top_left_x, top_left_y), label, font=font, fill=255)
        origin_image.save(output_path)



MODEL_PATH = "./model/yolov3_yuv.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

def main():
    #程序执行时带图片目录参数
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    #实例化分类检测,传入om模型存放路径,模型输入宽高参数
    detect = ObjectDetect(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    #推理初始化
    ret = detect.init()
    check_ret("ObjectDetect.init ", ret)
    
    #从参数获取图片存放目录,逐张图片推理
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    #创建目录，保存推理结果
    if not os.path.isdir('./outputs'):
        os.mkdir('./outputs')

    for image_file in images_list:
        #读入图片
        image = AclImage(image_file)
        #对图片预处理
        resized_image = detect.pre_process(image)
        print("pre process end")
        #推理图片
        result = detect.inference(resized_image)
        #对推理结果进行处理
        detect.post_process(result, image, image_file)

if __name__ == '__main__':
    main()
 