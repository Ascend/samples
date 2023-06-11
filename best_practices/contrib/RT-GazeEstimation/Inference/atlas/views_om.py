from django.shortcuts import render, redirect, HttpResponse

import sys
import os
import acl
import torch
import numpy as np
import cv2
from PIL import Image
import torchvision.transforms as T
import time

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, "../common/"))
sys.path.append(os.path.join(path, "../common/acllite"))

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource

transform = T.Compose([
    T.Lambda(lambda x: cv2.resize(x, (224, 224))),
    T.Lambda(lambda x: x[:, :, ::-1].copy()),  # BGR -> RGB
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224,
                                                 0.225]),  # RGB
])

# om模型路径
model_path = os.path.join(path,"../model/resnet18_batch1.om")

# ret = acl.init()
acl_resource = AclLiteResource()
acl_resource.init()
context, ret = acl.rt.get_context()

model = AclLiteModel(model_path)

def inferenceByom(request):
    if request.method == "GET":
        return HttpResponse("24.3413 2.3532")
    elif request.method == "POST":
        face_image = request.FILES.getlist("face")[0]  # 获取文件list列表并取第一个元素，即上传的图像InMemoryUploadedFile类
        face_image = Image.open(face_image)  # 将InMemoryUploadedFile转换为Image PIL类型
        face_image = cv2.cvtColor(np.asarray(face_image), cv2.COLOR_RGB2BGR)  # 将PIL类型变为cv2类型
        face_image = transform(face_image).unsqueeze(0).numpy()  #变为numpy
        
        ret = acl.rt.set_context(context)  

        # t1 = time.time()
        prediction = model.execute([face_image, ])
        # time.sleep(1)
        # print("性能：",(time.time()-t1)*1000,"毫秒")
        gaze_str = str(prediction[0][0][0]) + " " + str(prediction[0][0][1])

        # return HttpResponse("24.3413 2.3532")
        return HttpResponse(gaze_str)

