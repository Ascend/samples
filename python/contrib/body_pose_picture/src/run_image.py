
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

import os
import cv2
import numpy as np
import argparse
import sys
sys.path.append("../../../common/")
SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
print(SRC_PATH)
from utils.model_processor import ModelProcessor
from acllite_resource import AclLiteResource 


MODEL_PATH = os.path.join(SRC_PATH, "../model/" + "OpenPose_light.om")
print("MODEL_PATH:", MODEL_PATH)
DATA_PATH = os.path.join(SRC_PATH, "../data/" + "test.jpg")
Output_PATH = os.path.join(SRC_PATH, "../output/")

def main(model_path, frames_input_src, output_dir):
    """main"""
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
    # Read the image input using OpenCV
    img_original = cv2.imread(frames_input_src)

    ## Model Prediction ##
    # model_processor.predict: processing + model inference + postprocessing
    # canvas: the picture overlayed with human body joints and limbs
    canvas = model_processor.predict(img_original)

    # Save the detected results
    cv2.imwrite(os.path.join(output_dir, "test_output.jpg"), canvas)
    

if __name__ == '__main__':   
    description = 'Load a model for human pose estimation'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--model', type=str, default=MODEL_PATH)
    parser.add_argument('--frames_input_src', type=str, default=DATA_PATH, help="Directory path for image")
    parser.add_argument('--output_dir', type=str, default=Output_PATH, help="Output Path")

    args = parser.parse_args()
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    main(args.model, args.frames_input_src, args.output_dir)
