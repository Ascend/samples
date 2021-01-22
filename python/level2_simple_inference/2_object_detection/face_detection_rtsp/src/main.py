import datetime
import time
import acl
import configparser
import sys
import os

cur_file_dir = os.path.dirname(os.path.abspath(__file__))
 
sys.path.append(cur_file_dir + "/../../../../common")

import atlas_utils.video as video
from atlas_utils.constants import *
from atlas_utils.utils import *
from acl_resource import AclResource
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_logger import log_error, log_info

from vgg_ssd import VggSsd
from preprocess import Preprocess
from postprocess import DetectData, Postprocess


MODEL_PATH = "../model/face_detection.om"
MODEL_WIDTH = 304
MODEL_HEIGHT = 300
FACE_DETEC_CONF="../scripts/face_detection.conf"

def create_threads(detector):
    config = configparser.ConfigParser()
    config.read(FACE_DETEC_CONF)
    
    video_decoders = []
    for item in config['videostream']:
        preprocesser = Preprocess(config['videostream'][item], 
                                  len(video_decoders),
                                  MODEL_WIDTH, MODEL_HEIGHT)
        video_decoders.append(preprocesser)

    rtsp_num = len(video_decoders)
    if rtsp_num == 0:
        log_error("No video stream name or addr configuration in ",
                  FACE_DETEC_CONF)
        return None, None

    postprocessor = Postprocess(detector)

    display_channel = int(config['display']['channel'])
    if (display_channel is None) or (display_channel >= rtsp_num):
        log_info("No video to display, display configuration: ", 
                 config['display']['channel'])
    else:
        video_decoders[display_channel].set_display(True)
        ret = postprocessor.create_presenter_channel(FACE_DETEC_CONF)
        if ret == False:
            log_error("Create presenter channel failed")
            return None, None
        
    return video_decoders, postprocessor


def main():
    #初始化acl
    acl_resource = AclResource()
    acl_resource.init()   

    #创建一个检测网络实例,当前使用vgg_ssd网络.当更换检测网络时,在此实例化新的网络
    detector = VggSsd(acl_resource, MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)

    video_decoders, postprocessor = create_threads(detector)
    if video_decoders is None:
        log_error("Please check the configuration in %s is valid"
                  %(FACE_DETEC_CONF))
        return
    
    while True:
        all_process_fin = True
        for decoder in video_decoders:
            ret, data = decoder.get_data()
            if ret == False:                
                log_info("Read data ret ", ret)
                continue
            
            if data:
                detect_results = detector.execute(data)
                postprocessor.process(data, detect_results)
                
            all_process_fin = False

        if all_process_fin:
            log_info("all video decoder finish")
            break

    postprocessor.exit()

    log_info("sample execute end")  


if __name__ == '__main__':
    main()
