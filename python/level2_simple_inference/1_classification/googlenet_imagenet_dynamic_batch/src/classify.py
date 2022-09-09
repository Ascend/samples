import sys
import os
import numpy
import acl

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from acllite_utils import check_ret
from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont

class Classify(object):
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self.total_buffer = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height

        self._model = AclLiteModel(model_path)
        self._dvpp = AclLiteImageProc(acl_resource)
        print("The App arg is __init__")

    def __del__(self):
        if self.total_buffer:
            acl.rt.free(self.total_buffer)  
        if self._dvpp:
            del self._dvpp
        print("[Sample] class Samle release source success")

    def pre_process(self, image):
        yuv_image = self._dvpp.jpegd(image)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image
    
    def batch_process(self, resized_image_list):
        resized_img_data_list = []
        resized_img_size = resized_image_list[0].size
        total_size = MAX_BATCH * resized_img_size
        stride = 0
        for resized_image in resized_image_list:
            resized_img_data_list.append(resized_image.data())
        self.total_buffer, ret = acl.rt.malloc(total_size, ACL_MEM_MALLOC_HUGE_FIRST)
        check_ret("acl.rt.malloc", ret)    
        for i in range(len(resized_image_list)):
            ret = acl.rt.memcpy(self.total_buffer + stride, resized_img_size,\
                        resized_img_data_list[i], resized_img_size,\
                        ACL_MEMCPY_DEVICE_TO_DEVICE)
            check_ret("acl.rt.memcpy", ret)
            stride += resized_img_size
        return total_size
    
    def inference(self, resized_image_list, batch):
        total_size = self.batch_process(resized_image_list)
        batch_buffer = {'data': self.total_buffer, 'size':total_size}
        return self._model._execute_with_dynamic_batch_size([batch_buffer, ], batch)
    
    def post_process(self, infer_output, batch_image_files, number_of_images):
        print("post process") 
        datas = infer_output[0]
        
        for number in range(number_of_images):
            data = datas[number]
            vals = data.flatten()
            top_k = vals.argsort()[-1:-6:-1]
            print("images:{}".format(batch_image_files[number]))
            print("======== top5 inference results: =============")
            for n in top_k:
                object_class = get_image_net_class(n)
                print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
            
            #Use Pillow to write the categories with the highest confidence on the image and save them locally
            if len(top_k):
                object_class = get_image_net_class(top_k[0])
                output_path = os.path.join("../out", os.path.basename(batch_image_files[number]))
                origin_img = Image.open(batch_image_files[number])
                draw = ImageDraw.Draw(origin_img)
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=20)
                draw.text((10, 50), object_class, font=font, fill=255)
                origin_img.save(output_path)

MODEL_PATH = "../model/googlenet_dynamic_batch.om"
MODEL_WIDTH = 224
MODEL_HEIGHT = 224
MAX_BATCH = 8

def main():
    """
    Program execution with picture directory parameters
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclLiteResource()
    acl_resource.init()
    #Instance classification detection, pass into the OM model storage path, model input width and height parameters
    classify = Classify(acl_resource, MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    #Create a directory to store the inference results
    if not os.path.isdir('../out'):
        os.mkdir('../out')

    resized_image_list = []
    batch_image_files = []
    batch_size = 2
    print(images_list[:2])
    for image_file in images_list[:2]:
        #Read the pictures
        image = AclLiteImage(image_file)
        image_dvpp = image.copy_to_dvpp()
        #preprocess image
        resized_image = classify.pre_process(image_dvpp)
        print("pre process end")

        batch_image_files.append(image_file) 
        resized_image_list.append(resized_image)

    #Reasoning pictures
    result = classify.inference(resized_image_list, batch_size)
    #process inference results
    classify.post_process(result, batch_image_files, batch_size)

if __name__ == '__main__':
    main()
 
