import sys
import time

import acl
import os
import numpy as np
from easydict import EasyDict as edict

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, "../common/"))
sys.path.append(os.path.join(path, "../common/acllite"))
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource

import reader


def gazeto3d(gaze):
    assert gaze.size == 2, "The size of gaze must be 2"
    gaze_gt = np.zeros([3])
    gaze_gt[0] = -np.cos(gaze[1]) * np.sin(gaze[0])
    gaze_gt[1] = -np.sin(gaze[1])
    gaze_gt[2] = -np.cos(gaze[1]) * np.cos(gaze[0])
    return gaze_gt


def angular(gaze, label):
    assert gaze.size == 3, "The size of gaze must be 3"
    assert label.size == 3, "The size of label must be 3"

    total = np.sum(gaze * label)
    return np.arccos(min(total / (np.linalg.norm(gaze) * np.linalg.norm(label)), 0.9999999)) * 180 / np.pi


if __name__ == '__main__':

    # om模型路径
    model_path = "/home/HwHiAiUser/WB/om/resnet18_batch1.om"

    val_data = edict({"image": "/home/HwHiAiUser/WB/ETH-XGaze/Image/valid_train",
                      "label": "/home/HwHiAiUser/WB/ETH-XGaze/Label/validation.label",
                      "header": True,
                      "name": "eth",
                      "isFolder": False
                      })
    batch_size_valid = 1

    testdataset = reader.loader(val_data, batch_size_valid, shuffle=False, num_workers=0)

    acl_resource = AclLiteResource()
    acl_resource.init()

    model = AclLiteModel(model_path)

    valid_angle = 0.0
    accs = 0
    count = 0

    # batch为1推理
    print("Starting inference")

    for j, (data, label) in enumerate(testdataset):
        face_image = data["face"].numpy()
        label_t = label[0].numpy()
        # print(face_image.shape,label_t.shape)
        # t1 = time.time()
        result = model.execute([face_image, ])
        # print()time.time()
        accs += angular(gazeto3d(result[0][0]), gazeto3d(label_t))
        count += 1

    valid_angle = accs / count

    # GPU: 5.216430800677054
    # NPU: 5.221126418183157
    print("valid angle on NPU(degree): ", valid_angle)
