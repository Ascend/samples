import os
import sys
import importlib

import numpy as np
import caffe
from amct_caffe.common.auto_calibration import AutoCalibrationEvaluatorBase
import amct_caffe.common.cmd_line_utils.arguments_handler as args_handler
import amct_caffe.common.cmd_line_utils.data_handler as data_handler



class ResNet50Evaluator(AutoCalibrationEvaluatorBase):
    def calibration(self,modified_model, modified_weights):
        net = caffe.Net(modified_model, modified_weights, caffe.TEST)
        for data_map in data_handler.load_data(
                   {'data': [1, 3, 224, 224]},
                    ['./data/image'],
                    ['float32'],
                    1):
            _ = net.forward(**data_map)



customize_evaluator=ResNet50Evaluator()