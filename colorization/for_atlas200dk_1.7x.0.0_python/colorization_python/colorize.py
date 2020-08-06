import cv2 as cv
import numpy as np
import os

import time

from constants import *
from acl_model import Model
from acl_resource import AclResource


MODEL_WIDTH = 224
MODEL_HEIGHT = 224
out_w = 56
out_h = 56
INPUT_DIR = './data/'
OUTPUT_DIR = './out/'
model_path = './model/colorization_yuv.om'


def preprocess(picPath):
    #读取图片,像素RGB分量采用float32类型
    bgr_img = cv.imread(picPath).astype(np.float32)
    #opencv读入的图片为(N)HWC,获取HW分量
    orig_shape = bgr_img.shape[:2]
    #对图片归一化处理
    bgr_img = bgr_img / 255.0
    #将图片转换为Lab空间
    lab_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2Lab)
    #获取L分量
    orig_l = lab_img[:,:,0]
    #如果L分量在内存中存放不连续,则转换为连续内存存放
    if not orig_l.flags['C_CONTIGUOUS']:
        orig_l = np.ascontiguousarray(orig_l)
    #将Lab图像缩放到模型输入要求大小
    lab_img = cv.resize(lab_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
    #获取缩放后的L分量,如果缩放后L不连续,转换为连续内存存放
    l_data = lab_img[:,:,0]
    if not l_data.flags['C_CONTIGUOUS']:
        l_data = np.ascontiguousarray(l_data)
    #L分量减均值
    l_data = l_data - 50
    return orig_shape, orig_l, l_data


def postprocess(result_list, pic, orig_shape, orig_l):
    #推理结果为预测的ab通道值,数据为NCHW,需要转换为opencv可以处理的NHWC
    result_list[0] = result_list[0].reshape(1,2,56,56).transpose(0,2,3,1)
    result_array = result_list[0][0]
    #将预测的ab通道值其缩放到原始图片尺寸
    ab_data = cv.resize(result_array, orig_shape[::-1])
    #将预测并缩放到原始尺寸的ab通道值和原始的L分量值合并,得到预测图像的Lab值
    result_lab = np.concatenate((orig_l[:,:,np.newaxis],ab_data),axis=2)
    #将Lab转换为RGBU888
    result_bgr = (255*np.clip(cv.cvtColor(result_lab, cv.COLOR_Lab2BGR),0,1)).astype('uint8')
    #将推理得到的图片数据写文件
    file_name = os.path.join(OUTPUT_DIR, 'out_'+pic)
    cv.imwrite(file_name, result_bgr)


def main():
    #创建输出图片存放目录
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    #acl资源初始化
    acl_resource = AclResource()
    acl_resource.init()
    #加载黑白图像上色模型
    model = Model(acl_resource, model_path)
    src_dir = os.listdir(INPUT_DIR)
    #从data目录逐张读取图片进行推理
    for pic in src_dir:
        #读取图片
        pic_path = os.path.join(INPUT_DIR, pic)
        #获取图片L通道值
        orig_shape, orig_l, l_data = preprocess(pic_path)
        #送进模型推理
        result_list = model.execute([l_data,])    
        #处理推理结果
        postprocess(result_list, pic, orig_shape, orig_l)
    print("Execute end")


if __name__ == '__main__':
    main()
