import os
import argparse
import numpy as np
import cv2
from torchvision import transforms
import tqdm
from PIL import Image


def preprocess(dataset_dir, save_path, args):

    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([.5, .5, .5], [.5, .5, .5]),
    ])

    img_dir = os.path.join(dataset_dir, 'JPEGImages')
    mask_dir = os.path.join(dataset_dir, 'SegmentationClass')
    test_img_lists = os.path.join(dataset_dir, 'ImageSets/Segmentation/test.txt')

    patch_size = 512

    num = 0

    with open(test_img_lists) as f:
        img_names = f.readlines()
        total_imgs = len(img_names)

    for idx, img_name in enumerate(img_names):
        img_name = img_name.split('.')[0]
        img = cv2.imread(os.path.join(img_dir, img_name + '.bmp'))
        mask = cv2.imread(os.path.join(mask_dir, img_name + '_mask.png'), 0)
        # 4 cls
        mask[(mask == 1) | (mask == 2)] = 0
        mask[mask != 0] -= 2
        mask[mask == 255] = -1

        h, w, c = img.shape
        pred = np.zeros((h, w), dtype=np.float32)
        pred_softmax = np.zeros((args.num_class, h, w), dtype=np.float32)

        for i in range(0, h, patch_size):
            for j in range(0, w, patch_size):
                patch = img[i: i + patch_size, j: j + patch_size, :]
                target = mask[i: i + patch_size, j: j + patch_size]
                inpu = Image.fromarray(cv2.cvtColor(patch, cv2.COLOR_BGR2RGB))
                inpu = transform(inpu).unsqueeze(0).numpy()
                

                inpu.tofile(os.path.join(save_path, img_name + f"_{num}.bin"))
                target.tofile(os.path.join(save_path + "_mask", img_name + f"_{num}.bin"))
                num += 1

parser = argparse.ArgumentParser(description='preprocess')
parser.add_argument("--dataset_dir", type=str, default="./data/20221011_casting_voc")
parser.add_argument("--save_path", type=str, default="./bin")
parser.add_argument("--num_class", type=int, default=4)
args_opt = parser.parse_args()

if not os.path.exists(args_opt.save_path):
    os.mkdir(args_opt.save_path)
if not os.path.exists(args_opt.save_path + "_mask"):
    os.mkdir(args_opt.save_path + "_mask")


preprocess(args_opt.dataset_dir, args_opt.save_path, args_opt)
