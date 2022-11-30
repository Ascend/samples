import cv2 as cv
import numpy as np
import os
import sys
import time
import acl
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))
from constants import NPY_BYTE
from constants import IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
import acllite_utils as utils
from acllite_imageproc import AclLiteImageProc
import draw_predict
from multiprocessing import Process,Queue
import random

MODEL_WIDTH = 513
MODEL_HEIGHT = 513
INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
model_path = '../model/deeplab_quant.om'
OUTPUT_PATH =  os.path.join(path, '../output/')


STATUS_PROC_INIT = 1
STATUS_PROC_RUNNING = 2
STATUS_PROC_EXIT = 3
STATUS_PROC_ERROR = 4

class ProcData():
    def __init__(self, result_list, pic, OUTPUT_DIR):
        self.result_list = result_list
        self.pic = pic
        self.OUTPUT_DIR = OUTPUT_DIR

@utils.display_time
def preandinfer(image_queue, images_list):
    acl_start = time.time()    
    #print('Start preandinfer ')
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    acl_resource = AclLiteResource()
    acl_resource.init()
    dvpp_ = AclLiteImageProc()
    model = AclLiteModel(model_path)
    print('------------------------------acl processing time', time.time() - acl_start)
    for pic in images_list:
        image = AclLiteImage(pic)
        image_dvpp = image.copy_to_dvpp()
        yuv_image = dvpp_.jpegd(image_dvpp)
        resized_image = dvpp_.resize(yuv_image, MODEL_WIDTH, MODEL_HEIGHT)
        result_list = model.execute([resized_image,])
        data = ProcData(result_list, pic, OUTPUT_DIR) 
        image_queue.put(data)
    post_num = 6
    while(post_num):
        post_num -= 1
        data = "Post process thread exit"
        image_queue.put(data)
    print('End preandinfer')
    print('------------------------------preandinfer time', time.time() - acl_start)

@utils.display_time
def postprocess(image_queue,num):
    print('Start postprocess ', num)
    ret = True
    while ret: 
        if image_queue.empty():
            continue
        data = image_queue.get()
        if isinstance(data, ProcData):
            draw_predict.draw_label(data.pic, data.result_list[0].squeeze(), data.OUTPUT_DIR)
            print('End postprocess', num)
        elif isinstance(data, str):
            print(data)
            ret = False
        else: 
            print("post process thread receive unkonow data")

@utils.display_time
def main():
    images_list = [os.path.join(INPUT_DIR, img)
                   for img in os.listdir(INPUT_DIR)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    print(images_list)
    image_queue= Queue()
    start = time.time()
    proc_preprocess = Process(target = preandinfer,args = (image_queue,images_list))
    proc_postprocess_1 = Process(target = postprocess,args = (image_queue,1))
    proc_postprocess_2 = Process(target = postprocess,args = (image_queue,2))

    proc_preprocess.start()
    proc_postprocess_1.start()
    proc_postprocess_2.start()

    proc_preprocess.join()
    proc_postprocess_1.join()
    proc_postprocess_2.join()
    print('------------------------------processing time', time.time() - start)
    print("Execute end")



if __name__ == '__main__':
    main()

