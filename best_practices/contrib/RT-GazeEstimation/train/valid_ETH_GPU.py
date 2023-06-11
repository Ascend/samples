import timm
import numpy as np
import torch
from easydict import EasyDict as edict
import reader.reader as reader


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
  return np.arccos(min(total/(np.linalg.norm(gaze)* np.linalg.norm(label)), 0.9999999))*180/np.pi


def test_ETH(net,testdataset):
    net.eval()
    valid_angle = 0.0
    accs = 0
    count = 0
        # Testing --------------------------------------------------------------
    with torch.no_grad():
        for j, (data, label) in enumerate(testdataset):

            gts = label.cuda()
            data["face"] = data["face"].cuda()
            results = net(data["face"])

            for k, result in enumerate(results):

                result = result.cpu().detach().numpy()
                gt = gts[k].cpu().numpy()
                accs += angular(gazeto3d(gt),gazeto3d(result))

                count += 1
            # print("valid_angle",accs/count)
        valid_angle = accs/count
    return valid_angle


pth = "../data/resnet18/Iter_10_resnet18.pth"
val_data = edict({"image": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Image/train",
                  "label": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Label/validation.label",
                  "header": True,
                  "name": "eth",
                  "isFolder": False
                  })
batch_size_valid = 64

net = timm.create_model("resnet18", pretrained=False, num_classes=2)
net.load_state_dict(torch.load(pth, map_location='cpu'))
net.cuda()
print("load successful!!")

testdataset = reader.loader(val_data, batch_size_valid, shuffle=False, num_workers=0)
print("read over!")

angle = test_ETH(net, testdataset)

# GPU: 5.216430800677054
# NPU: 5.221126418183157
print("valid angle on GPU(degree): ", angle)