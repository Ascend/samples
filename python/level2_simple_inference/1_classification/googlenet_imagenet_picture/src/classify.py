import sys
import os
import acl

path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/atlas_utils"))

from utils import *
from constants import *
from acl_dvpp import Dvpp
from acl_model import Model
from acl_image import AclImage
from acl_resource import AclResource
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont

class Classify(object):
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = Dvpp(acl_resource)
        self._model = Model(model_path)

    def __del__(self):
        if self._dvpp:
            del self._dvpp
        print("[Sample] class Samle release source success")

    def pre_process(self, image):
        yuv_image = self._dvpp.jpegd(image)
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        return self._model.execute([resized_image, ])

    def post_process(self, infer_output, image_file):
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-6:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_image_net_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        #使用pillow，将置信度最高的类别写在图片上，并保存到本地
        if len(top_k):
            object_class = get_image_net_class(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype(os.path.join(SRC_PATH, "SourceHanSansCN-Normal.ttf"), size=30)
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/googlenet_yuv.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224

def main():
    #程序执行时带图片目录参数
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclResource()
    acl_resource.init()
    #实例化分类检测,传入om模型存放路径,模型输入宽高参数
    classify = Classify(acl_resource, MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #从参数获取图片存放目录,逐张图片推理
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #创建目录，保存推理结果
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))

    for image_file in images_list:
        #读入图片
        image = AclImage(image_file)
        image_dvpp = image.copy_to_dvpp()
        #对图片预处理
        resized_image = classify.pre_process(image_dvpp)
        print("pre process end")
        #推理图片
        result = classify.inference(resized_image)
        #对推理结果进行处理
        classify.post_process(result, image_file)

if __name__ == '__main__':
    main()
 
