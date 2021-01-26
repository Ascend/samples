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


class ObjectDetect(object):

    """
    Perform model loading,
    Initialization, reasoning process
    """
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._image_info_size = None
        self._model = None 
        self.run_mode = None


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
        """
        Initialize  ACL resource
        """
        
        self._init_resource() 
        self._dvpp = Dvpp(self.stream, self.run_mode)

        #Initialize DVPP
        ret = self._dvpp.init_resource()
        if ret != SUCCESS:
            print("Init dvpp failed")
            return FAILED
        
        #load model
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
        print ("\n")
        print(box_info[0:6*box_num].reshape(6, box_num))
        scalex = origin_img.width / self._model_width
        scaley = origin_img.height / self._model_height
        output_path = os.path.join("../outputs", os.path.basename(image_file))
        origin_image = Image.open(image_file)
        draw = ImageDraw.Draw(origin_image)
        font = ImageFont.truetype("../SourceHanSansCN-Normal.ttf", size=30)
        print("images:{}".format(image_file))
        print("======== inference results: =============")
        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            label = labels[ids]
            score = box_info[4 * int(box_num)+n]
            top_left_x = box_info[0 * int(box_num)+n] * scalex
            top_left_y = box_info[1 * int(box_num)+n] * scaley
            bottom_right_x = box_info[2 * int(box_num) + n] * scalex
            bottom_right_y = box_info[3 * int(box_num) + n] * scaley
            print(" % s: class % d, box % d % d % d % d, score % f" % (
                label, ids, top_left_x, top_left_y, 
                bottom_right_x, bottom_right_y, score))
            draw.line([(top_left_x, top_left_y), (bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), \
            (top_left_x, bottom_right_y), (top_left_x, top_left_y)], fill=(0, 200, 100), width=3)
            draw.text((top_left_x, top_left_y), label, font=font, fill=255)
        origin_image.save(output_path)



MODEL_PATH = "../model/yolov3_yuv.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

def main():
    """
    Program execution with picture directory parameters
    """
    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    #Instance classification detection, pass into the OM model storage path, model input width and height parameters
    detect = ObjectDetect(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    #Inference initialization
    ret = detect.init()
    check_ret("ObjectDetect.init ", ret)
    
    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    #Create a directory to store the inference results
    if not os.path.isdir('../outputs'):
        os.mkdir('../outputs')

    for image_file in images_list:
        #read picture
        image = AclImage(image_file)
        #preprocess image
        resized_image = detect.pre_process(image)
        print("pre process end")
        #reason pictures
        result = detect.inference(resized_image)
        #process resresults
        detect.post_process(result, image, image_file)

if __name__ == '__main__':
    main()
 
