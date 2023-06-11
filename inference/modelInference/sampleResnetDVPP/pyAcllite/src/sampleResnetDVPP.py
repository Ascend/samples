import os
import numpy as np
from acllite_imageproc import AclLiteImage
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
from label import label

class sample_resnet_dvpp(object):
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

    def get_result(self):
        # do data processing with softmax
        res = np.array(self.result).flatten()
        res = np.exp(res)

        # print top 5 classes
        top_k = res.argsort()[-1:-6:-1]
        total = np.sum(res)
        print("======== top5 inference results: =============")
        for j in top_k:
            label_class = ""
            label_string = label.get(str(j))
            if label_string:
                list_iterator = iter(range(len(label_string)))
                for i in list_iterator:
                    if i == len(label_string) - 1:
                        label_class += label_string[i]
                else:
                    label_class += label_string[i] + "," + "\t"
                print("label:%d confidence:%f class:%s" % (j, res[j]/total, label_class))
            else:
                raise Exception("the key of label is not exist")

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
    model_width = 224
    model_height = 224
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "../model/resnet50.om")
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
    net = sample_resnet_dvpp(model_path)
    net.init_resource()
    for image in all_path:
        net.process_input(image, model_width, model_height)
        net.inference()
        net.get_result()
    print("*****run finish******")
    net.release_resource()
