
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

import presenteragent
from utils.model_processor import ModelProcessor
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/" + "OpenPose_light.om")
BODYPOSE_CONF="body_pose.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720
DATA_PATH = '../data/yoga.mp4'


def main(model_path, frames_input_src, output_dir, is_presenter_server):
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
    # Read the video input using OpenCV
    cap = cv2.VideoCapture(frames_input_src)

    ## Set Output ##
    if is_presenter_server:
        # if using presenter server, then open the presenter channel
        chan = presenteragent.presenter_channel.open_channel(BODYPOSE_CONF)
        if chan is None:
            print("Open presenter channel failed")
            return
    else:
        # if saving result as video file (mp4), then set the output video writer using opencv
        video_output_path = '{}/demo-{}-{}.mp4'.format(
            output_dir, os.path.basename(frames_input_src), str(random.randint(1, 100001)))
        video_writer = cv2.VideoWriter(video_output_path, 0x7634706d, 25,
                                                (1280, 720))
        if video_writer is None:
            print('Error: cannot get video writer from openCV')


    while(cap.isOpened()):
        ## Read one frame of the input video ## 
        ret, img_original = cap.read()

        if not ret:
            print('Cannot read more, Reach the end of video')
            break

        ## Model Prediction ##
        # model_processor.predict: processing + model inference + postprocessing
        # canvas: the picture overlayed with human body joints and limbs
        canvas = model_processor.predict(img_original)
        
        ## Present Result ##
        if is_presenter_server:
            # convert to jpeg image for presenter server display
            _, jpeg_image = cv2.imencode('.jpg', canvas)
            # construct AclLiteImage object for presenter server
            jpeg_image = AclLiteImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)
            # send to presenter server
            chan.send_detection_data(img_original.shape[0], img_original.shape[1], jpeg_image, [])

        else:
            # save to video
            video_writer.write(canvas)
    
    # release the resources
    cap.release()
    if not is_presenter_server:
        video_writer.release()
   

    

if __name__ == '__main__':   

    description = 'Load a model for human pose estimation'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--model', type=str, default=MODEL_PATH)
    parser.add_argument('--frames_input_src', type=str, default=DATA_PATH, help="Directory path for video.")
    parser.add_argument('--output_dir', type=str, default='../output', help="Output Path")
    parser.add_argument('--is_presenter_server', type=bool, default=False, 
                        help="Display on presenter server or save to a video mp4 file (T/F)")
    args = parser.parse_args()
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    main(args.model, args.frames_input_src, args.output_dir, args.is_presenter_server)
