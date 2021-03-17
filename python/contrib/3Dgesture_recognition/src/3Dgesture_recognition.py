#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import numpy as np
import struct
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/atlas_utils"))

import numpy as np
import acl
import base64
import atlas_utils.utils as utils
from PIL import Image, ImageDraw, ImageFont
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/frozen_model.om")
MODEL_WIDTH = 112
MODEL_HEIGHT = 112
image_net_classes = [
   "Action1", "Action2","Action3", "Action4","Action5", "Action6","Action7","Action8",
     "Action9", "Action10"]

def get_image_net_class(class_id):
    if class_id >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[class_id]


def post_process(infer_output):
    print("post process")
    data = infer_output[0]
    vals = data.flatten()
    top_k = vals.argsort()[-1:-9:-1]    

    for n in top_k:
        object_class = get_image_net_class(n)
        print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
       
    return 

def main():    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclResource()
    acl_resource.init()
    model = Model(MODEL_PATH)            
               
    data_raw = np.fromfile("./data/test_float32_actiontype7.bin",dtype =np.float32)
    input_data =  data_raw.reshape(16,MODEL_WIDTH,MODEL_HEIGHT,3).copy()
    result = model.execute([input_data,])  
    post_process(result)    

    print("process  end")
if __name__ == '__main__':
    main()



