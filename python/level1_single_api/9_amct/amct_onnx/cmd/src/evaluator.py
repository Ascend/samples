import amct_onnx.common.cmd_line_utils.data_handler as data_handler
import onnxruntime as ort
import amct_onnx as amct
class Evaluator():
    def calibration(self, modified_model):
        ort_session = ort.InferenceSession(modified_model, amct.AMCT_SO)
        for data_map in data_handler.load_data(
                    {'input': [16, 3, 224, 224]},
                    ['data/images/'],
                    ['float32'],
                    1):
            _ = ort_session.run(['output'], data_map)

customize_evaluator = Evaluator()
