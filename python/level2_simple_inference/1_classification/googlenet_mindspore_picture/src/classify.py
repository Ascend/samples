"""classify.py"""
import sys
import os
import acl

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from googlenet_classes import get_googlenet_class
from PIL import Image, ImageDraw, ImageFont
import numpy as np

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/googlenet.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224

class Classify(object):
    """classify"""
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._model = AclLiteModel(model_path)

    def __del__(self):
        print("[Sample] class Samle release source success")

    def pre_process(self, image):
        """preprocess"""
        input_image = Image.open(image)
        input_image = input_image.resize((224, 224))
        # hwc
        img = np.array(input_image)
        height = img.shape[0]
        width = img.shape[1]
        h_off = int((height - 224) / 2)
        w_off = int((width - 224) / 2)
        crop_img = img[h_off:height - h_off, w_off:width - w_off, :]
        # rgb to bgr
        print("crop shape = ", crop_img.shape)
        img = crop_img[:, :, ::-1]
        shape = img.shape
        print("img shape = ", shape)
        img = img.astype("float32")
        img[:, :, 0] *= 0.003922
        img[:, :, 1] *= 0.003922
        img[:, :, 2] *= 0.003922
        img[:, :, 0] -= 0.4914
        img[:, :, 0] = img[:, :, 0] / 0.2023
        img[:, :, 1] -= 0.4822
        img[:, :, 1] = img[:, :, 1] / 0.1994
        img[:, :, 2] -= 0.4465
        img[:, :, 2] = img[:, :, 2] / 0.2010
        img = img.reshape([1] + list(shape))
        # nhwc -> nchw
        result = img.transpose([0, 3, 1, 2]).copy()
        return result

    def inference(self, resized_image):
        """inference"""
        return self._model.execute([resized_image, ])

    def post_process(self, infer_output, image_file):
        """postprocess"""
        print("post process")
        data = infer_output[0]
        print("data shape = ", data.shape)
        vals = data.flatten()
        max = 0
        sum = 0
        for i in range(0, 10):
            if vals[i] > max:
                max = vals[i] 
        for i in range(0, 10):
            vals[i] = np.exp(vals[i] - max)
            sum += vals[i]
        for i in range(0, 10):
            vals[i] /= sum
        print("vals shape = ", vals.shape)
        top_k = vals.argsort()[-1:-6:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_googlenet_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        #using pillow, the category with the highest confidence is written on the image and saved locally
        if len(top_k):
            object_class = get_googlenet_class(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../out"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=20)
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)

def main():
    """
    Program execution with picture directory parameters
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid, eg: python3.6 classify.py ../data/")
        exit(1)
    acl_resource = AclLiteResource()
    acl_resource.init()
    #Instantiation classification detection, incoming om model path, model input width and height parameters
    classify = Classify(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #Get the picture storage directory from the parameters, and infer picture by picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #Create a directory and save the infer results
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))

    for image_file in images_list:
        #preprocess image
        resized_image = classify.pre_process(image_file)
        print("pre process end")
        #inference
        result = classify.inference(resized_image)
        #post process
        classify.post_process(result, image_file)

if __name__ == '__main__':
    main()
 
