import os
import numpy as np
import cv2
from acllite_imageproc import AclLiteImage
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
from acllite_logger import log_info


class sample_crowd_counting(object):
    def __init__(self, model_path):
        self.model_path = model_path    # string
        self.resource = None
        self.dvpp = None
        self.model = None
        self.resized_image = None
        self.result = None

    def init_resource(self):
        # init acl resource
        self.resource = AclLiteResource()
        self.resource.init()

        # init dvpp resource
        self.dvpp = AclLiteImageProc(self.resource)

        # load model from file
        self.model = AclLiteModel(self.model_path)

    def process_input(self, input_path, model_width, model_height):
        # read image from file
        self.image = AclLiteImage(input_path)

        # memory copy from host to dvpp
        image_input = self.image.copy_to_dvpp()

        # decode image from JPEGD format to YUV
        yuv_image = self.dvpp.jpegd(image_input)

        # execute resize
        self.resized_image = self.dvpp.resize(yuv_image, model_width, model_height)

    def inference(self):
        # inference
        self.result = self.model.execute([self.resized_image])

    def get_result(self, src_path):
        # do data processing
        outputs, _ = self.result
        output_image = outputs[0, 0]
        
        # How many people are there in total
        out_number = int(np.sum(outputs).item())
        
        # map the output_image to 0-255
        output_image = (output_image - output_image.min()) / (output_image.max() - output_image.min() + 1e-5)
        output_image = (output_image * 255).astype(np.uint8)
        output_image = cv2.applyColorMap(output_image, cv2.COLORMAP_JET)
        
        # read source image from path
        src_image = cv2.imread(src_path)
        
        # Zoom the width and height of output_image to the same as source image
        resized_image = cv2.resize(output_image, (np.shape(src_image)[1], np.shape(src_image)[0]))
        
        # combine the resized image and source image together
        combine_image = cv2.addWeighted(src_image, 1, resized_image, 0.8, 0.0)
        
        # put text to result_image
        cv2.putText(combine_image, str(out_number), (30, 60), cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 8)
        
         # write output image
        out_path = src_path.replace('data', 'out')
        cv2.imwrite(out_path, combine_image)
 

    def release_resource(self):
        # release resource includes acl resource, data set and unload model
        self.dvpp.__del__()
        self.model.__del__()
        self.resource.__del__()
        AclLiteResource.__del__ = lambda x: 0
        AclLiteImage.__del__ = lambda x: 0
        AclLiteImageProc.__del__ = lambda x: 0
        AclLiteModel.__del__ = lambda x: 0

if __name__ == '__main__':
    model_width = 1280
    model_height = 850
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "../model/CrowdCounting.om")
    if not os.path.exists(model_path):
        raise Exception("the model is not exist")

    # read all file path of image
    images_path = os.path.join(current_dir, "../data")
    if not os.path.exists(images_path):
        raise Exception("the directory is not exist")
    all_path = []
    for path in os.listdir(images_path):
        if path != '.keep':
            total_path = os.path.join(images_path, path)
            all_path.append(total_path)
    if len(all_path) == 0:
        raise Exception("the directory is empty, please download image")

    # inference
    net = sample_crowd_counting(model_path)
    net.init_resource()
    for image in all_path:
        net.process_input(image, model_width, model_height)
        net.inference()
        net.get_result(image)
    net.release_resource()
    log_info("success")
