import sys
sys.path.append("../../../../common")
sys.path.append("../")
project_path = sys.path[0] + "/../"
sys.path.append(project_path)
import datetime
import numpy as np
from cameracapture import CameraCapture
import presenteragent.presenter_channel as presenter_channel
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
from yolov3 import Yolov3

MODEL_PATH = project_path + "/model/object_detection.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416
OBJECT_DETEC_CONF= project_path + "/scripts/object_detection.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

def main():
    """main"""
    #Initialize acl
    acl_resource = AclLiteResource()
    acl_resource.init()
    #Create a detection network instance, currently using the vgg_ssd network. 
    # When the detection network is replaced, instantiate a new network here
    detect = Yolov3(acl_resource, MODEL_WIDTH, MODEL_HEIGHT)
    #Load offline model
    model = AclLiteModel(MODEL_PATH)
    #Connect to the presenter server according to the configuration, 
    # and end the execution of the application if the connection fails
    chan = presenter_channel.open_channel(OBJECT_DETEC_CONF)
    if chan is None:
        print("Open presenter channel failed")
        return
    #Open the CARAMER0 camera on the development board
    cap = CameraCapture(0)

    image_info = np.array([MODEL_WIDTH, MODEL_HEIGHT,
                          MODEL_WIDTH, MODEL_HEIGHT],
                          dtype = np.float32)
    while True:
        #Read a picture from the camera
        image = cap.read()
        if image is None:
            print("Get memory from camera failed")
            break
        
        #The detection network processes images into model input data
        model_input = detect.pre_process(image)
        if model_input is None:
            print("Pre process image failed")
            break
        #Send data to offline model inference
        result = model.execute([model_input, image_info])
        #Detecting network analysis inference output
        jpeg_image, detection_list = detect.post_process(result, image)
        if jpeg_image is None:
            print("The jpeg image for present is None")
            break
        
        chan.send_detection_data(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, 
                                 jpeg_image, detection_list)


if __name__ == '__main__':
    main()
