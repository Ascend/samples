"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
"""
import sys
import cv2 as cv
import numpy as np
import os
import re
import time
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/atlas_utils"))

from acl_model import Model
from acl_image import AclImage
from acl_resource import AclResource
from atlas_utils import presenteragent

MODEL_WIDTH = 224
MODEL_HEIGHT = 224
model_path = '../model/colorization.om'
COLORIZATION_CONF ="../scripts/param.conf"

def preprocess(frame):
    """
    Preprocess the images before they are sent into the model
    """
    #Read the image
    bgr_img = frame.astype(np.float32)

    #Opencv reads the picture as (N) HWC to get the HW value
    orig_shape = bgr_img.shape[:2]

    #Normalize the picture
    bgr_img = bgr_img / 255.0

    #Convert the picture to Lab space
    lab_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2Lab)

    #Gets the L
    orig_l = lab_img[:, :, 0]

    if not orig_l.flags['C_CONTIGUOUS']:
        orig_l = np.ascontiguousarray(orig_l)

    #resize
    lab_img = cv.resize(lab_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)

    l_data = lab_img[:, :, 0]
    if not l_data.flags['C_CONTIGUOUS']:
        l_data = np.ascontiguousarray(l_data)

    #The L-part minus the average
    l_data = l_data - 50

    return orig_shape, orig_l, l_data


def postprocess(result_list, orig_shape, orig_l):
    """
    Post process the images after model reasoning
    """
    result_list[0] = result_list[0].reshape(1, 2, 56, 56).transpose(0, 2, 3, 1)
    result_array = result_list[0][0]

    #Scales the predicted ab channel value to the original picture size
    ab_data = cv.resize(result_array, orig_shape[::-1])

    #get the Lab value of the predicted image
    result_lab = np.concatenate((orig_l[:, :, np.newaxis], ab_data), axis = 2)

    #Convert Lab to RGBU888
    result_bgr = (255 * np.clip(cv.cvtColor(result_lab, cv.COLOR_Lab2BGR), 0, 1)).astype('uint8')

    return cv.imencode('.jpg', result_bgr)[1]


def main():
    """
    acl resource initialization
    """
    acl_resource = AclResource()
    acl_resource.init()

    #load model
    model = Model(model_path)
    chan = presenteragent.presenter_channel.open_channel(COLORIZATION_CONF)
    if chan is None:
        print("Open presenter channel failed")
        return

    lenofUrl = len(sys.argv)

    if lenofUrl <= 1:
        print("[ERROR] Please input mp4/Rtsp URL")
        exit()
    elif lenofUrl >= 3:
        print("[ERROR] param input Error")
        exit()
    URL = sys.argv[1]
    URL1 = re.match('rtsp://', URL)
    URL2 = re.search('.mp4', URL)

    if URL1 is None and URL2 is None:
        print("[ERROR] should input correct URL")
        exit()

    cap = cv.VideoCapture(URL)

    #Gets the total frames
    frames_num = cap.get(7)
    currentFrames = 0

    while True:
        #read image
        ret, frame = cap.read()

        if ret is not True:
            print("read None image, break")
            break

        if currentFrames == frames_num - 1:
            currentFrames = 0
            cap.set(1, 0)

        currentFrames += 1

        #Gets the L channel value
        orig_shape, orig_l, l_data = preprocess(frame)
        result_list = model.execute([l_data,])
        result_jpeg = postprocess(result_list, orig_shape, orig_l)
        chan.send_image(orig_shape[0], orig_shape[1], result_jpeg)

if __name__ == '__main__':
    main()
