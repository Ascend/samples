import os
import cv2
import numpy as np
import sys
import math

def resize_image(im, max_side_len=2400, resize_h=768, resize_w=768):
    '''
    因为NPU不支持动态shape，所以这里不能简单的根据模型的输入的shape 768*768，
    直接进行resize，
    否则会因为图像的失真而导致精度的下降
    '''
    h, w, _ = im.shape
    print("origin_h:%d, resize_h:%d"%(h,resize_h))
    print("origin_w:%d, resize_w:%d"%(w,resize_w))

    if h <= resize_h and w <= resize_w:
        im = cv2.copyMakeBorder(im,0,resize_h-h,0,resize_w-w,cv2.BORDER_CONSTANT,value=(0,0,0))
        ratio_h = 1
        ratio_w = 1
    else:
        ratio_w = ratio_h = resize_h/max(h,w)
        im = cv2.resize(im, (math.floor(w*ratio_w), math.floor(h*ratio_h)))
        im = cv2.copyMakeBorder(im, 0, resize_h - math.floor(h*ratio_h), 0, resize_w - math.floor(w*ratio_w), cv2.BORDER_CONSTANT, value=(0, 0, 0))

    print("ratio_h=ratio_w=",ratio_h)
    im = np.asarray(im, dtype='float32')
    #cv2.imshow('image',im)
    #cv2.waitKey(0)
    return im, (ratio_h, ratio_w)

if __name__ == "__main__":
    src_path = sys.argv[1]
    dst_path = sys.argv[2]
    files = os.listdir(src_path)
    files.sort()
    for file in files:
        if file.endswith('.JPEG') or file.endswith('.jpg') or file.endswith('.png'):
            src = src_path + "/" + file
            print("start to process %s" % src)
            img_org= cv2.imread(src)[:, :, ::-1]
            #img_org = cv2.imread(src)
            im_resized, (ratio_h, ratio_w) = resize_image(img_org)
            print(im_resized.shape)
            im_resized.tofile(dst_path + "/" + file + ".bin")
