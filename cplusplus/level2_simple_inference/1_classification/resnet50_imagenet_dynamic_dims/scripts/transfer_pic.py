import os
import logging
import numpy as np
from PIL import Image
logging.getLogger().setLevel(logging.INFO)


def process224(input_path):
    try:
        input_image = Image.open(input_path)
        input_image = input_image.resize((256, 256))
        # hwc
        img = np.array(input_image)
        height = img.shape[0]
        width = img.shape[1]
        h_off = int((height-224)/2)
        w_off = int((width-224)/2)
        crop_img = img[h_off:height-h_off, w_off:width-w_off, :]
        # rgb to bgr
        img = crop_img[:, :, ::-1]
        shape = img.shape
        img = img.astype("float16")
        img[:, :, 0] -= 104
        img[:, :, 1] -= 117
        img[:, :, 2] -= 123
        img = img.reshape([1] + list(shape))
        result = img.transpose([0, 3, 1, 2])
        output_name = "./data_224_224/" + input_path.split('.')[0] + ".bin"
        result.tofile(output_name)
    except Exception as except_err:
        logging.error(except_err)
        return 1
    else:
        return 0


def process200(input_path):
    try:
        input_image = Image.open(input_path)
        input_image = input_image.resize((256, 256))
        # hwc
        img = np.array(input_image)
        height = img.shape[0]
        width = img.shape[1]
        h_off = int((height-200)/2)
        w_off = int((width-200)/2)
        crop_img = img[h_off:height-h_off, w_off:width-w_off, :]
        # rgb to bgr
        img = crop_img[:, :, ::-1]
        shape = img.shape
        img = img.astype("float16")
        img[:, :, 0] -= 104
        img[:, :, 1] -= 117
        img[:, :, 2] -= 123
        img = img.reshape([1] + list(shape))
        result = img.transpose([0, 3, 1, 2])
        output_name = "./data_200_200/" + input_path.split('.')[0] + ".bin"
        result.tofile(output_name)
    except Exception as except_err:
        logging.error(except_err)
        return 1
    else:
        return 0


if __name__ == "__main__":
    count_ok = 0
    count_ng = 0
    images = os.listdir(r'./')
    for image_name in images:
        if not image_name.endswith("jpg"):
            continue
        logging.info("start to process image %s....", image_name)
        ret = process224(image_name)
        if ret == 0:
            logging.info("process224 image %s successfully", image_name)
            count_ok = count_ok + 1
        elif ret == 1:
            logging.info("failed to process224 image %s", image_name)
            count_ng = count_ng + 1
        
        ret = process200(image_name)
        if ret == 0:
            logging.info("process200 image %s successfully", image_name)
            count_ok = count_ok + 1
        elif ret == 1:
            logging.info("failed to process200 image %s", image_name)
            count_ng = count_ng + 1
    logging.info("%d images in total, %d images process successfully, %d images process failed",
          count_ok + count_ng, count_ok, count_ng)
