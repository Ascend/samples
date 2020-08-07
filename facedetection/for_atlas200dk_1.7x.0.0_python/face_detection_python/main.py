import datetime

from atlas_utils.camera import Camera
from atlas_utils import presenteragent
from acl_model import Model
from acl_resource import AclResource
from vgg_ssd import VggSsd

MODEL_PATH = "./model/face_detection.om"
MODEL_WIDTH = 304
MODEL_HEIGHT = 300
FACE_DETEC_CONF="./script/face_detection.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

def main():
    #初始化acl
    acl_resource = AclResource()
    acl_resource.init()
    #创建一个检测网络实例,当前使用vgg_ssd网络.当更换检测网络时,在此实例化新的网络
    detect = VggSsd(acl_resource, MODEL_WIDTH, MODEL_HEIGHT)
    #加载离线模型
    model = Model(acl_resource, MODEL_PATH)
    #根据配置连接presenter server,连接失败则结束应用的执行
    chan = presenteragent.presenter_channel.open_channel(FACE_DETEC_CONF)
    if chan == None:
        print("Open presenter channel failed")
        return
    #打开开发板上CARAMER0摄像头
    cap = Camera(0)
    while True:
        #从摄像头读入一帧图片
        image = cap.read()
        #检测网络将图片处理为模型输入数据
        model_input = detect.pre_process(image)
        if model_input == None:
            print("Pre process image failed")
            continue
        #将数据送入离线模型推理
        result = model.execute(model_input)
        #检测网络解析推理输出
        jpeg_image, detection_list = detect.post_process(result, image)
        if jpeg_image == None:
            print("The jpeg image for present is None")
            continue
        chan.send_detection_data(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, 
                                 jpeg_image, detection_list)


if __name__ == '__main__':
    main()
