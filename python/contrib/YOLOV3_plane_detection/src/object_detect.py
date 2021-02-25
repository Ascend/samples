"""
-*- coding:utf-8 -*-
"""
import sys
sys.path.append("../../../common")
sys.path.append("../")
import os
import numpy as np
import acl
import cv2
import re
import json

import atlas_utils.utils as utils
from PIL import Image, ImageDraw, ImageFont
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

labels = ["plane"]

MODEL_PATH = "../model/yolov3.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

def construct_image_info():
    """construct image info"""
    image_info = np.array([MODEL_WIDTH, MODEL_HEIGHT, 
                           MODEL_WIDTH, MODEL_HEIGHT], 
                           dtype = np.float32) 
    return image_info


def pre_process(image, dvpp):
    """preprocess"""
    image_input = image.copy_to_dvpp()
    yuv_image = dvpp.jpegd(image_input)
    print("decode jpeg end")
    resized_image = dvpp.resize(yuv_image, 
                    MODEL_WIDTH, MODEL_HEIGHT)
    print("resize yuv end")
    return resized_image


def post_process(infer_output, origin_img, image_file):
    """post process"""
    print("post process")
    print(infer_output[1])
    # box_num indicates the number of targets detected in the picture
    box_num = infer_output[1][0, 0]
    print("box num ", box_num)
    box_info = infer_output[0].flatten()
    print ("\n")
    print(box_info[0:6 * box_num].reshape(6, box_num))
    scalex = utils.align_up128(origin_img.width) / MODEL_WIDTH
    scaley = utils.align_up16(origin_img.height) / MODEL_HEIGHT
    output_path = os.path.join("./outputs", os.path.basename(image_file))
    origin_image = Image.open(image_file)
    draw = ImageDraw.Draw(origin_image)
    font = ImageFont.truetype("SourceHanSansCN-Normal.ttf", size=30)
    print("images:{}".format(image_file))

    # Create a list to save Json results
    obj_res = []

    # Inference result output
    print("======== inference results: =============")
    for n in range(int(box_num)):
        ind = int(box_info[5 * int(box_num) + n])
        label = labels[ind]
        score = box_info[4 * int(box_num) + n]
        top_left_x = box_info[0 * int(box_num) + n] * scalex
        top_left_y = box_info[1 * int(box_num) + n] * scaley
        bottom_right_x = box_info[2 * int(box_num) + n] * scalex
        bottom_right_y = box_info[3 * int(box_num) + n] * scaley
        # Output the target name, category number, coordinate position, and detection rate in turn
        print("%s: class %d, box %d %d %d %d, score %f" % (
            label, ind, top_left_x, top_left_y, 
            bottom_right_x, bottom_right_y, score))
        # Mark the test results on the picture
        draw.line([(top_left_x, top_left_y), (bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), 
        (top_left_x, bottom_right_y), (top_left_x, top_left_y)], fill=(0, 200, 100), width=2)
        draw.text((top_left_x, top_left_y), label, font=font, fill=255)

        # json data
        obj = {}
        obj['label'] = label
        obj['score'] = score

        points = {}
        points['lx'] = top_left_x
        points['ly'] = top_left_y
        points['rx'] = bottom_right_x
        points['ry'] = bottom_right_y
        obj['points'] = points 

        obj_res.append(obj)

    #Save the final test result picture
    origin_image.save(output_path)
    return obj_res


def post_process_big(infer_output, origin_img, image_file, out_target):
    """post process big"""
    print("post process")
    print(infer_output[1])
    box_num = infer_output[1][0, 0]
    print("box num ", box_num)
    box_info = infer_output[0].flatten()
    print ("\n")
    print(box_info[0:6 * box_num].reshape(6, box_num))
    scalex = utils.align_up128(origin_img.width) / MODEL_WIDTH
    scaley = utils.align_up16(origin_img.height) / MODEL_HEIGHT
    output_path = os.path.join(out_target, os.path.basename(image_file))
    origin_image = Image.open(image_file)
    draw = ImageDraw.Draw(origin_image)
    font = ImageFont.truetype("../SourceHanSansCN-Normal.ttf", size=30)
    print("images:{}".format(image_file))
    print("======== inference results: =============")
   
    imagename = get_file_name(image_file) 

    # Get the number in the name of the cropped picture
    x = imagename.split("_")
    row_num = (int)(x[0]) 
    y = x[1].split(".")
    col_num = (int)(y[0])     

    obj_res = []

    for n in range(int(box_num)):
        ind = int(box_info[5 * int(box_num)+n])
        label = labels[ind]
        score = box_info[4 * int(box_num)+n]
        top_left_x = box_info[0 * int(box_num)+n] * scalex
        top_left_y = box_info[1 * int(box_num)+n] * scaley
        bottom_right_x = box_info[2 * int(box_num)+n] * scalex
        bottom_right_y = box_info[3 * int(box_num)+n] * scaley
        print("%s: class %d, box %d %d %d %d, score %f" % (
            label, ind, top_left_x, top_left_y, 
            bottom_right_x, bottom_right_y, score))
        draw.line([(top_left_x, top_left_y), (bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), 
        (top_left_x, bottom_right_y), (top_left_x, top_left_y)], fill=(0, 200, 100), width=2)
        draw.text((top_left_x, top_left_y), label, font=font, fill=255)

        # Big picture mapping coordinates
        big_lx = top_left_x + (col_num - 1) * 832
        big_ly = top_left_y + (row_num - 1) * 832
        big_rx = bottom_right_x + (col_num - 1) * 832
        big_ry = bottom_right_y + (row_num - 1) * 832

        #json data
        obj = {}
        obj['label'] = label
        obj['score'] = score

        points = {}
        points['lx'] = big_lx
        points['ly'] = big_ly
        points['rx'] = big_rx
        points['ry'] = big_ry
        obj['points'] = points 

        obj_res.append(obj)

    origin_image.save(output_path)
    return obj_res


def get_file_name(originalFilepath):
    """get file name"""
    # Use regular expressions to get the image name
    end = "/"
    filestr = originalFilepath[originalFilepath.rfind(end):]
    tailpart='/(.+?\\.jpg)'
    imagename=re.compile(tailpart).findall(filestr)
    imagename=str(imagename[0])
    return imagename


def crop_picture(filename, crop_target):
    """crop picture"""
    # Size to be divided
    cut_width = 832
    cut_length = 832
    # Read the picture to be divided, and its size and other data
    picture = cv2.imread(filename, -1)
    (width, length, depth) = picture.shape
    # Preprocessing generates 0 matrix
    pic = np.zeros((cut_width, cut_length, depth))
    # Calculate the number of horizontal and vertical that can be divided
    num_width = int(width / cut_width)
    num_length = int(length / cut_length)
    # Generate and save the cropped picture
    for i in range(0, num_width):
        for j in range(0, num_length):
            pic = picture[i * cut_width: (i + 1) * cut_width, j * cut_length: (j + 1) * cut_length, :]
            result_path = os.path.join(crop_target, '{}_{}.jpg'.format(i + 1, j + 1))
            cv2.imwrite(result_path, pic)

  
def merge_picture(merge_path, filename):
    """merge picture"""
    # The folder of the divided pictures, and the folder to be saved after stitching
    pic_path = merge_path
    merge_target = "./outputs"
    num_width_list = []
    num_lenght_list = []
    picture_names = os.listdir(pic_path)
    if len(picture_names) == 0:
        print("no file")

    else:
        # Get the size of the divided image
        imgpath = os.path.join(pic_path, '1_1.jpg')
        img_1_1 = cv2.imread(imgpath)
        (width, length, depth) = img_1_1.shape
        # Split the name to get the number of rows and columns
        for picture_name in picture_names:
            num_width_list.append(int(picture_name.split("_")[0]))
            num_lenght_list.append(int((picture_name.split("_")[-1]).split(".")[0]))
        num_width = max(num_width_list)
        num_length = max(num_lenght_list)
        # Pre-generate spliced pictures
        splicing_pic = np.zeros((num_width * width, num_length * length, depth))
        for i in range(1, num_width + 1):
            for j in range(1, num_length + 1):
                imgpartpath = os.path.join(pic_path, '{}_{}.jpg'.format(i, j))
                img_part = cv2.imread(imgpartpath, -1)
                splicing_pic[width * (i - 1): width * i, length * (j - 1): length * j, :] = img_part
        
        resultimg = filename + '.jpg'
        targetpath = os.path.join(merge_target, resultimg)
        cv2.imwrite(targetpath, splicing_pic)


def main():
    """main"""
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    acl_resource = AclResource()
    acl_resource.init()
    model = Model(MODEL_PATH)
    dvpp = Dvpp(acl_resource)
    
    image_dir = sys.argv[1] 
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    # Create a directory to save inference results
    if not os.path.isdir('./outputs'):
        os.mkdir('./outputs')
    
    # Create a directory to save the intermediate results of the large image detection
    if not os.path.isdir('./bigpic'):
        os.mkdir('./bigpic')

    # Create a directory to save the results of the big picture cropping inference
    outCrop = os.path.join('./bigpic', 'output')
    if not os.path.isdir(outCrop):
        os.mkdir(outCrop)

    # Create a directory, save the large and cropped pictures
    cropImg = os.path.join('./bigpic', 'cropimg')
    if not os.path.isdir(cropImg):
        os.mkdir(cropImg)
   
    image_info = construct_image_info()
    
    for image_file in images_list: 
        imagename = get_file_name(image_file)          
        tempfile = os.path.splitext(imagename)[0]

        imgdic = {}
        imgdic['name'] = imagename
        obj_res = []

        img = cv2.imread(image_file, -1)
        (width, height, depth) = img.shape
        if width > 1000 and height > 1000:
            # Create a directory to save the divided pictures of each big picture
            crop_target = os.path.join(cropImg, tempfile)
            if not os.path.isdir(crop_target):
                os.mkdir(crop_target)

            # Create a directory to save the inference results of each large image
            out_target = os.path.join(outCrop, tempfile)
            if not os.path.isdir(out_target):
                os.mkdir(out_target)

            # Large image clipping function
            crop_picture(image_file, crop_target)
            cropimg_list = [os.path.join(crop_target, imgs)
                   for imgs in os.listdir(crop_target)
                   if os.path.splitext(imgs)[1] in const.IMG_EXT] 

            # After the execution of the crop function is over, 
            # the small picture after the big picture crop should be saved in a folder crop_target
            for cropimg_file in cropimg_list:
                print("the crop filename is :\t", cropimg_file)
                image = AclImage(cropimg_file)
                resized_image = pre_process(image, dvpp)
                result = model.execute([resized_image, image_info])  
                resdic = post_process_big(result, image, cropimg_file, out_target)
                obj_res.extend(resdic)

            imgdic['object_result'] = obj_res

            merge_picture(out_target, tempfile)

        # Read in the picture, if the picture size is less than 1000x1000, 
        # it will be read in and processed normally
        else:  
            print("detect the small picture")
            image = AclImage(image_file)
            resized_image = pre_process(image, dvpp)
            print("pre process end")
            result = model.execute([resized_image, image_info])  
            resdic = post_process(result, image, image_file)
            obj_res.extend(resdic)
            imgdic['object_result'] = obj_res


if __name__ == '__main__':
    main()
 