import cv2
from PIL import Image
import random
import numpy as np


def get_edge_cor(img):
    gray_defect_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary_defect_img = cv2.threshold(gray_defect_img, 50, 255, cv2.THRESH_BINARY_INV)
    edge = cv2.Canny(binary_defect_img, 30, 200)
    line = edge[:, img.shape[0] // 2]
    line = list(line)
    top = 0
    bottom = len(line) - 1
    result = []
    while (top < bottom):
        if (line[top] == 255):
            pass
        else:
            top += 1
        if (line[bottom] == 255):
            pass
        else:
            bottom -= 1
        if line[top] == 255 and line[bottom] == 255:
            result.append(top)
            result.append(bottom)
            break
    return result


def rand(a=0, b=1):
    return np.random.rand() * (b - a) + a


def filp_image(image, mask):
    if rand() > 0.5:
        image = cv2.flip(image, 0)  # 垂直翻转
        mask = cv2.flip(mask, 0)
    if rand() > 0.5:
        image = cv2.flip(image, 1)  # 水平翻转
        mask = cv2.flip(mask, 1)

    return image, mask


def rotate_image(image, mask):
    if rand() > 0.5:
        image = Image.fromarray(image)
        mask = Image.fromarray(mask)
        angle = random.randint(-30, 30)
        image = image.rotate(angle)
        mask = mask.rotate(angle)
        image = np.array(image, np.uint8)
        mask = np.array(mask, np.uint8)
    return image, mask


def resize_image(image, mask):
    if rand() > 0.5:
        h, w, c = image.shape[:]
        ratio = 0.12 * random.randint(6, 10)
        new_h, new_w = h, ratio * 2 / np.log10(w) * w
        new_h, new_w = int(new_h), int(new_w)
        image = cv2.resize(image, (new_h, new_w), cv2.INTER_CUBIC)
        mask = cv2.resize(mask, (new_h, new_w), cv2.INTER_NEAREST)
    return image, mask