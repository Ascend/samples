"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
import sys
import os
import acl

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont

class Classify(object):
    """
    Classify
    """
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = AclLiteImageProc(acl_resource)
        self._model = AclLiteModel(model_path)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("[Sample] class Samle release source success")

    def pre_process(self, image):
        """
        pre_process
        """    
        yuv_image = self._dvpp.jpegd(image)
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        """
        inference
        """
        return self._model.execute([resized_image, ])

    def post_process(self, infer_output, image_file):
        """
        post_process
        """    
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-6:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_image_net_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        #using pillow, the category with the highest confidence is written on the image and saved locally
        if len(top_k):
            object_class = get_image_net_class(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../out"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=20)
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/InceptionV3.om")
MODEL_WIDTH = 336
MODEL_HEIGHT = 336

def main():

    """
    Program execution with picture directory parameters
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclLiteResource()
    acl_resource.init()
    #Instantiation classification detection, incoming om model path, model input width and height parameters
    classify = Classify(acl_resource, MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #Get the picture storage directory from the parameters, and infer picture by picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #Create a directory and save the infer results
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))

    for image_file in images_list:
        #read the picture
        image = AclLiteImage(image_file)
        image_dvpp = image.copy_to_dvpp()
        #preprocess image
        resized_image = classify.pre_process(image_dvpp)
        print("pre process end")
        #inference
        result = classify.inference(resized_image)
        #post process
        classify.post_process(result, image_file)

if __name__ == '__main__':
    main()
 
