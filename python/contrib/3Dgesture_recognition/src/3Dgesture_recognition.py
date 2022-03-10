"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import struct
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/acllite"))

import numpy as np
import acl
import base64
import acllite_utils as utils
from PIL import Image, ImageDraw, ImageFont
import constants as const
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/3d_gesture_recognition.om")
MODEL_WIDTH = 112
MODEL_HEIGHT = 112
image_net_classes = [
   "Action1", "Action2", "Action3", "Action4", "Action5", "Action6", "Action7", "Action8",
     "Action9", "Action10"]

def get_image_net_class(class_id):
    """
    get_image_net_class
    """
    if class_id >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[class_id]

def post_process(infer_output, data_file):
    """
    post_process
    """
    print("post process")
    data = infer_output[0]
    vals = data.flatten()
    top_k = vals.argsort()[-1:-9:-1]    

    for n in top_k:
        object_class = get_image_net_class(n)
        print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
    
    (filepath, tempfilename) = os.path.split(data_file)
    (filename, extension) = os.path.splitext(tempfilename)
    object_class = get_image_net_class(top_k[0])
    output_path = os.path.join(os.path.join(SRC_PATH, "../out"), filename + ".txt")
    with open(output_path, "w", encoding="utf-8") as fp:
        fp.write(object_class)
       
    return 

def main():    
    """
    main
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclLiteResource()
    acl_resource.init()
    model = AclLiteModel(MODEL_PATH)             

    data_dir = sys.argv[1]  
    data_list = [os.path.join(data_dir, testdata)
                   for testdata in os.listdir(data_dir)
                   if os.path.splitext(testdata)[1] in ['.bin']]

    #Create a directory to store the inference results
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))

    for data_file in data_list:     
        data_raw = np.fromfile(data_file, dtype = np.float32)
        input_data =  data_raw.reshape(16, MODEL_WIDTH, MODEL_HEIGHT, 3).copy()
        result = model.execute([input_data,])  
        post_process(result, data_file)    

    print("process  end")
if __name__ == '__main__':
    main()



