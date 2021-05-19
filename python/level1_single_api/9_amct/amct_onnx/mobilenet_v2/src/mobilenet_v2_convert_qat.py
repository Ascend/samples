"""
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
"""


import os
import numpy as np
from PIL import Image # pylint: disable=E0401
import onnxruntime as ort # pylint: disable=E0401

import amct_onnx as amct # pylint: disable=E0401


SIDE = 224
PATH = os.path.realpath('./')
MODEL_DIR = os.path.join(PATH, 'model')
OUTPUTS = os.path.join(PATH, 'outputs/convert_qat')


def main(): # pylint: disable=too-mant-locals, not-context-manager
    """main"""
    # Step one, inferece the original model to obtain the original
    # precision before conversion.
    image_path = os.path.join(PATH, 'data/classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float32) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])
    image_test = image_test.astype(np.float32).transpose([0, 3, 1, 2])

    model_path = os.path.join(MODEL_DIR, 'mobilenetv2_qat.onnx')
    ort_session = ort.InferenceSession(model_path, amct.AMCT_SO)
    output = ort_session.run(None, {'input:0': image_test})
    origin_prediction = output[0]

    # Step two, convert QAT pb_model to ascend quantized model and save
    # it. The ascend quantized model can be used for simulating test on
    # CPU/GPU and evaluate the accuracy of quantized model, and it can
    # also be used for ATC to generate the model running on Ascend AI
    # Processor.
    record_path = os.path.join(OUTPUTS, 'record.txt')
    amct.convert_qat_model(
        model_file=model_path, save_path=os.path.join(OUTPUTS, 'mobilenet_v2'), record_file=record_path)

    # Step three, reload and test the quantized model.
    fake_quant_model = os.path.join(OUTPUTS, 'mobilenet_v2_fake_quant_model.onnx')
    ort_session = ort.InferenceSession(fake_quant_model, amct.AMCT_SO)
    output = ort_session.run(None, {'input:0': image_test})
    convert_prediction = output[0]

    print(
        'Origin Model Prediction:\n',
        '\tcategory index: {}\n'.format(origin_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(origin_prediction.max()))
    print(
        'Quantized Model Prediction:\n',
        '\tcategory index: {}\n'.format(convert_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(convert_prediction.max()))


if __name__ == '__main__':
    main()
