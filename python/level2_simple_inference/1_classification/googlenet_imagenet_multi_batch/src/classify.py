import sys
import os
import numpy
import acl

from utils import *
from acl_dvpp import Dvpp
from acl_model import Model
from acl_image import AclImage
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont

class Classify(object):
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = None
        self.total_buffer = None

    def __del__(self):
        if self.total_buffer:
            acl.rt.free(self.total_buffer)  
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
    Initialize ACL resource
    """
        
        self._init_resource() 
        self._dvpp = Dvpp(self.stream, self.run_mode)

        #Initialize DVPP
        ret = self._dvpp.init_resource()
        if ret != SUCCESS:
            print("Init dvpp failed")
            return FAILED
        
        #Load model
        self._model = Model(self.run_mode, self._model_path)
        ret = self._model.init_resource()
        if ret != SUCCESS:
            print("Init model failed")
            return FAILED

        return SUCCESS

    def pre_process(self, image):
        yuv_image = self._dvpp.jpegd(image)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image
    
    def batch_process(self, resized_image_list, batch):
        resized_img_data_list = []
        resized_img_size = resized_image_list[0].size
        total_size = batch * resized_img_size
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
        total_size = self.batch_process(resized_image_list, batch)
        return self._model.execute(self.total_buffer, total_size)
    
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
                output_path = os.path.join("../outputs", os.path.basename(batch_image_files[number]))
                origin_img = Image.open(batch_image_files[number])
                draw = ImageDraw.Draw(origin_img)
                font = ImageFont.truetype("SourceHanSansCN-Normal.ttf", size=30)
                draw.text((10, 50), object_class, font=font, fill=255)
                origin_img.save(output_path)

MODEL_PATH = "../model/googlenet_yuv.om"
MODEL_WIDTH = 224
MODEL_HEIGHT = 224
#Batch number to 10
BATCH = 10
def main():
    """
    Program execution with picture directory parameters
    """
    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    #Instance classification detection, pass into the OM model storage path, model input width and height parameters
    classify = Classify(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    #Inference initialization
    ret = classify.init()
    check_ret("Classify.init ", ret)
    
    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #Create a directory to store the inference results
    if not os.path.isdir('../outputs'):
        os.mkdir('../outputs')
    
    resized_image_list = []
    batch_image_files = []
    num = 0
    batch_amount = len(images_list) // BATCH
    left =  len(images_list) % BATCH

    for image_file in images_list:
        num += 1
        #Read the pictures
        image = AclImage(image_file)
        #preprocess image
        resized_image = classify.pre_process(image)
        print("pre process end")
        
        batch_image_files.append(image_file) 
        resized_image_list.append(resized_image)
        if batch_amount > 0:
            #Each set of BATCH pictures, reasoning and post-processing
            if num == BATCH:
                #Reasoning pictures
                result = classify.inference(resized_image_list, BATCH)
                #process inference results
                classify.post_process(result, batch_image_files, BATCH)
                batch_amount -= 1
                num = 0
                batch_image_files = []
                resized_image_list = []
        else:
            #remaining images are inferred and post-processed
            if num == left:
                #Reasoning pictures
                result = classify.inference(resized_image_list, BATCH)
                #The inference results are processed
                classify.post_process(result, batch_image_files, left)
            
if __name__ == '__main__':
    main()
 
