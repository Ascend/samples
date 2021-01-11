import datetime
import cv2 as cv
import sys
import re
import atlas_utils.video as video

from atlas_utils.camera import Camera
from atlas_utils import presenteragent
from acl_model import Model
from acl_resource import AclResource
from vgg_ssd import VggSsd
#from atlas_utils.acl_logger import log_error, log_info

MODEL_PATH = "./model/yolo3_resnet18_yuv.om"
MODEL_WIDTH = 640
MODEL_HEIGHT = 352
MASK_DETEC_CONF="./script/mask_detection.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

def main():
    acl_resource = AclResource()
    acl_resource.init()
    #创建一个检测网络实例,当前使用vgg_ssd网络.当更换检测网络时,在此实例化新的网络
    detect = VggSsd(acl_resource, MODEL_WIDTH, MODEL_HEIGHT)
    #加载离线模型
    model = Model(MODEL_PATH)
    #根据配置连接presenter server,连接失败则结束应用的执行
    
    chan = presenteragent.presenter_channel.open_channel(MASK_DETEC_CONF)
    if chan == None:
        print("Open presenter channel failed")
        return


    lenofUrl = len(sys.argv)

    if lenofUrl <= 1:
        print("[ERROR] Please input mp4/Rtsp URL")
        exit()
    elif lenofUrl >= 3:
        print("[ERROR] param input Error")
        exit()
    URL = sys.argv[1]
    URL1 = re.match('rtsp://', URL)
    URL2 = re.search('.mp4', URL)

    if URL1 is None and URL2 is None:
        print("[ERROR] should input correct URL")
        exit()
    cap = video.AclVideo(URL)
    
    while True:
        #从摄像头读入一帧图片
        ret,image = cap.read()
        if ret != 0:
            print("read None image, break")
            break

        #检测网络将图片处理为模型输入数据
        model_input = detect.pre_process(image)
        if model_input == None:
            print("Pre process image failed")
            break
        
        #将数据送入离线模型推理
        result = model.execute(model_input)
        if result is None:
            print("execute mode failed")
            break
        
        #检测网络解析推理输出
        jpeg_image, detection_list = detect.post_process(result, image)
        if jpeg_image == None:
            print("The jpeg image for present is None")
            break

        chan.send_detection_data(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                 jpeg_image, detection_list)
    
        

if __name__ == '__main__':
    main()

