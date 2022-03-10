import sys
sys.path.append("../../../../common")
sys.path.append("../")
import os
import numpy as np
import cv2 as cv
import acl
import acllite_utils as utils
from acllite_imageproc import AclLiteImageProc
import constants as const
from acllite_model import AclLiteModel
from acllite_utils import display_time 
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

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

MODEL_PATH = "../model/yolov3_tf_aipp.om"
MODEL_PATH2 = "../model/yolov3_output_op.om"
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

@display_time
def pre_process(image, dvpp):
    """preprocess"""
    image_input = image.copy_to_dvpp()
    yuv_image = dvpp.jpegd(image_input)
    print("decode jpeg end")
    resized_image = dvpp.crop_and_paste(yuv_image, image.width, image.height, MODEL_WIDTH, MODEL_HEIGHT)
    print("resize yuv end")
    return resized_image

@display_time
def post_process(infer_output, bgr_img, image_file):
    """postprocess"""
    print("post process")
    box_num = infer_output[1][0, 0]
    box_info = infer_output[0].flatten()
    scalex = bgr_img.shape[1] / MODEL_WIDTH
    scaley = bgr_img.shape[0] / MODEL_HEIGHT
    if scalex > scaley:
        scaley =  scalex
    output_path = os.path.join("../out", os.path.basename(image_file))
    print("image file = ", image_file)
    for n in range(int(box_num)):
        ids = int(box_info[5 * int(box_num) + n])
        label = labels[ids]
        score = box_info[4 * int(box_num)+n]
        top_left_x = box_info[0 * int(box_num)+n] * scaley
        top_left_y = box_info[1 * int(box_num)+n] * scaley
        bottom_right_x = box_info[2 * int(box_num) + n] * scaley
        bottom_right_y = box_info[3 * int(box_num) + n] * scaley
        print(" % s: class % d, box % d % d % d % d, score % f" % (
            label, ids, top_left_x, top_left_y, 
            bottom_right_x, bottom_right_y, score))
        cv.rectangle(bgr_img, (int(top_left_x), int(top_left_y)), 
                (int(bottom_right_x), int(bottom_right_y)), colors[n % 6])
        p3 = (max(int(top_left_x), 15), max(int(top_left_y), 15))
        cv.putText(bgr_img, label, p3, cv.FONT_ITALIC, 0.6, colors[n % 6], 1)

    output_file = os.path.join("../out", "out_" + os.path.basename(image_file))
    print("output:%s" % output_file)
    cv.imwrite(output_file, bgr_img)
    print("success!")

@display_time
def yolo_detectionoutput_inference(model, result_list0, result_list1, result_list2, image_info):    
    """yolo detectionoutput inference"""
    result_list0 = result_list0.transpose(0, 3, 1, 2).copy()
    result_list1 = result_list1.transpose(0, 3, 1, 2).copy()
    result_list2 = result_list2.transpose(0, 3, 1, 2).copy()
    return model.execute([result_list0, result_list1, result_list2, image_info])

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
    
    acl_resource = AclLiteResource()
    acl_resource.init()
    model = AclLiteModel(MODEL_PATH)
    model2 = AclLiteModel(MODEL_PATH2)
    dvpp = AclLiteImageProc(acl_resource)
    
    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]
    #Create a directory to store the inference results
    if not os.path.isdir('../out'):
        os.mkdir('../out')

    image_info = construct_image_info()

    for image_file in images_list:
        print("======== using dvpp preprocess begin: =============")
        #read picture
        image = AclLiteImage(image_file)
        bgr_img = cv.imread(image_file)
        #preprocess image
        resized_image = pre_process(image, dvpp)
        print("pre process end")
        #reason pictures
        result_list = model.execute([resized_image, ])    
        result_list2 = yolo_detectionoutput_inference(model2, 
                result_list[0], result_list[1], result_list[2], image_info)
        #process resresults
        post_process(result_list2, bgr_img, image_file)
        print("======== using op postprocess end: =============")

if __name__ == '__main__':
    main()
