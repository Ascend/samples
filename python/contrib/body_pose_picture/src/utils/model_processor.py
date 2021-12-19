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
import sys
from utils.pose_decode import decode_pose
from acllite_model import AclLiteModel

heatmap_width = 92
heatmap_height = 92

class ModelProcessor(object):
    """acl model wrapper"""
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params
        self._model_width = params['width']
        self._model_height = params['height']

        assert 'model_dir' in params and params['model_dir'] is not None, 'Review your param: model_dir'
        assert os.path.exists(params['model_dir']), "Model directory doesn't exist {}".format(params['model_dir'])
            
        # load model from path, and get model ready for inference
        self.model = AclLiteModel(params['model_dir'])

    def predict(self, img_original):
        """run predict"""
        #preprocess image to get 'model_input'
        model_input = self.preprocess(img_original)

        # execute model inference
        result = self.model.execute([model_input]) 
        # postprocessing: use the heatmaps (the output of model) to get the joins and limbs for human body
        # Note: the model has multiple outputs, here we used a simplified method, which only uses heatmap for body joints
        #       and the heatmap has shape of [1,14], each value correspond to the position of one of the 14 joints. 
        #       The value is the index in the 92*92 heatmap (flatten to one dimension)
        heatmaps = result[0]
        # calculate the scale of original image over heatmap, Note: image_original.shape[0] is height
        scale = np.array([img_original.shape[1] / heatmap_width, img_original.shape[0]/ heatmap_height])

        canvas = decode_pose(heatmaps[0], scale, img_original)

        return canvas

    def preprocess(self, img_original):
        """
        preprocessing: resize image to model required size, and normalize value between [0,1]
        """
        scaled_img_data = cv2.resize(img_original, (self._model_width, self._model_height))
        preprocessed_img = np.asarray(scaled_img_data, dtype=np.float32) / 255.
        
        return preprocessed_img
