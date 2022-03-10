
"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import random
import os
import cv2
import numpy as np
import argparse
import sys
sys.path.append("../../../common/")

from utils.model_processor import ModelProcessor
from cameracapture import CameraCapture
from presenteragent import presenter_channel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/" + "OpenPose_light.om")
BODYPOSE_CONF="body_pose.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

def main(model_path):
    """main"""
    ## Initialization ##
    #initialize acl runtime 
    acl_resource = AclLiteResource()
    acl_resource.init()

    ## Prepare Model ##
    # parameters for model path and model inputs
    model_parameters = {
        'model_dir': model_path,
        'width': 368, # model input width      
        'height': 368, # model input height
    }
    # perpare model instance: init (loading model from file to memory)
    # model_processor: preprocessing + model inference + postprocessing
    model_processor = ModelProcessor(acl_resource, model_parameters)
    
    ## Get Input ##
    # Initialize Camera
    cap = CameraCapture(camera_id = 0, fps = 10)

    ## Set Output ##
    # open the presenter channel

    chan = presenter_channel.open_channel(BODYPOSE_CONF)
    if chan is None:
        print("Open presenter channel failed")
        return



    while True:
        ## Read one frame from Camera ## 
        img_original = cap.read()
        if not img_original:
            print('Error: Camera read failed')
            break
        # Camera Input (YUV) to RGB Image
        img_original = img_original.byte_data_to_np_array()
        img_original = YUVtoRGB(img_original)
        # img_original = cv2.flip(img_original, 1)

        ## Model Prediction ##
        # model_processor.predict: processing + model inference + postprocessing
        # canvas: the picture overlayed with human body joints and limbs
        canvas = model_processor.predict(img_original)
        
        ## Present Result ##
        # convert to jpeg image for presenter server display
        _, jpeg_image = cv2.imencode('.jpg', canvas)
        # construct AclLiteImage object for presenter server
        jpeg_image = AclLiteImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)
        # send to presenter server
        chan.send_detection_data(img_original.shape[0], img_original.shape[1], jpeg_image, [])

    # release the resources
    cap.release()


def YUVtoRGB(byteArray):
    """Convert YUV format image to RGB"""
    e = 1280 * 720
    Y = byteArray[0:e]
    Y = np.reshape(Y, (720, 1280))

    s = e
    V = byteArray[s::2]
    V = np.repeat(V, 2, 0)
    V = np.reshape(V, (360, 1280))
    V = np.repeat(V, 2, 0)

    U = byteArray[s + 1::2]
    U = np.repeat(U, 2, 0)
    U = np.reshape(U, (360, 1280))
    U = np.repeat(U, 2, 0)

    RGBMatrix = (np.dstack([Y, U, V])).astype(np.uint8)
    RGBMatrix = cv2.cvtColor(RGBMatrix, cv2.COLOR_YUV2RGB, 3)
    return RGBMatrix
   

if __name__ == '__main__':   

    description = 'Load a model for human pose estimation'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--model', type=str, default=MODEL_PATH)
    args = parser.parse_args()

    main(args.model)
