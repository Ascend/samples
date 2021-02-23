import sys
sys.path.append("../../../../common")
sys.path.append("../")
import os
import numpy as np
import acl
import atlas_utils.utils as utils
from PIL import Image, ImageDraw, ImageFont
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

labels = ["person",
        "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
        "pizza", "donut", "cake", "chair", "sofa", "potted plant", "bed", "dining table",
        "toilet", "TV monitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]

MODEL_PATH = "../model/yolov3_yuv.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

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
    """postprocess"""
    print("post process")
    print(infer_output[1])
    box_num = infer_output[1][0, 0]
    print("box num ", box_num)
    box_info = infer_output[0].flatten()
    print ("\n")
    print(box_info[0:6 * box_num].reshape(6, box_num))
    scalex = origin_img.width / MODEL_WIDTH
    scaley = origin_img.height / MODEL_HEIGHT
    output_path = os.path.join("../outputs", os.path.basename(image_file))
    origin_image = Image.open(image_file)
    draw = ImageDraw.Draw(origin_image)
    font = ImageFont.truetype("../SourceHanSansCN-Normal.ttf", size=30)
    print("images:{}".format(image_file))
    print("======== inference results: =============")
    for n in range(int(box_num)):
        ids = int(box_info[5 * int(box_num) + n])
        label = labels[ids]
        score = box_info[4 * int(box_num)+n]
        top_left_x = box_info[0 * int(box_num)+n] * scalex
        top_left_y = box_info[1 * int(box_num)+n] * scaley
        bottom_right_x = box_info[2 * int(box_num) + n] * scalex
        bottom_right_y = box_info[3 * int(box_num) + n] * scaley
        print(" % s: class % d, box % d % d % d % d, score % f" % (
            label, ids, top_left_x, top_left_y, 
            bottom_right_x, bottom_right_y, score))
        draw.line([(top_left_x, top_left_y), (bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), \
        (top_left_x, bottom_right_y), (top_left_x, top_left_y)], fill=(0, 200, 100), width=3)
        draw.text((top_left_x, top_left_y), label, font=font, fill=255)
    origin_image.save(output_path)


def construct_image_info():
    """construct image info"""
    image_info = np.array([MODEL_WIDTH, MODEL_HEIGHT, 
                           MODEL_WIDTH, MODEL_HEIGHT], 
                           dtype = np.float32) 
    return image_info


def main():
    """
    Program execution with picture directory parameters
    """
    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    acl_resource = AclResource()
    acl_resource.init()
    model = Model(MODEL_PATH)
    dvpp = Dvpp(acl_resource)
    
    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]
    #Create a directory to store the inference results
    if not os.path.isdir('../outputs'):
        os.mkdir('../outputs')

    image_info = construct_image_info()

    for image_file in images_list:
        #read picture
        image = AclImage(image_file)
        #preprocess image
        resized_image = pre_process(image, dvpp)
        print("pre process end")
        #reason pictures
        result = model.execute([resized_image, image_info])    
        #process resresults
        post_process(result, image, image_file)

if __name__ == '__main__':
    main()
 