import os
import cv2
import numpy as np
import skimage.io as io
import skimage
from PossionFusion.args import get_PF_args
import shutil
print("skimage:", skimage.__version__)  # '0.14.2'
color2index = {
    (0, 0, 0): 0,
    (0, 128, 0): 1,
    (0, 0, 128): 2
}


def rgb2mask(img):
    assert len(img.shape) == 3
    height, width, ch = img.shape
    assert ch == 3
    W = np.power(2, [[0], [1], [2]])
    # 416*416*3 x 3*1===>416*416*1===>(squeeze)=416*416
    ch_sum = img.dot(W).squeeze(-1)
    # 不同种类的物体得到的sum是不一样的
    # 遍历所有的sum值
    sum_value = np.unique(ch_sum)
    # 定义输出结果为单层的mask
    mask = np.zeros_like(ch_sum)
    for i, val in enumerate(sum_value):
        mask[ch_sum == val] = color2index[tuple(img[ch_sum == val][0])]
    return mask


def split_data(args):
    if not os.path.exists(args.normal_data):
        os.makedirs(args.normal_data)

    if not os.path.exists(args.defect_data):
        os.makedirs(args.defect_data)

    if not os.path.exists(args.mask_data):
        os.makedirs(args.mask_data)
    for dir in os.listdir(args.out):
        path=os.path.join(args.out,dir+'/')
        print(path)
        shutil.copy(path+'img.png',args.defect_data+f'/{dir}.png')
        shutil.copy(path + 'mask.png', args.mask_data + f'/{dir}.png')

if __name__ == "__main__":
    # json转化成图片后，该图片文件所在的地址
    args=get_PF_args()
    img_path = args.out
    for file in os.listdir(img_path):
        name = file.split('.')[0]
        path = os.path.join(img_path, name + '/label.png')
        img = cv2.imread(path)
        mask = rgb2mask(img)
        mask = mask.astype(np.uint8)
        mask[mask > 0] = 255
        io.imsave(os.path.join(img_path, name + '/mask.png'), mask)
    split_data(args)