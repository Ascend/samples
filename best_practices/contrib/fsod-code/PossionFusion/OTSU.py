import cv2
import numpy as np
import os


def get_mask(bin_img, file):
    kernel1 = np.ones((4, 4), np.uint8)
    kernel2 = np.ones((5, 5), np.uint8)
    # bin_img1= cv2.erode(bin_img,  kernel1, iterations=3)
    bin_img1 = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel1,6)
    bin_img1 = cv2.morphologyEx(bin_img1, cv2.MORPH_CLOSE, kernel2, 10)
    # bin_img1 = cv2.dilate(bin_img1, kernel, iterations=4)
    # cv2.imshow("bin_img", bin_img)
    # cv2.imshow("bin_img1", bin_img1)
    cv2.imwrite(os.path.join("extracted_mask", file), bin_img1)
    # cv2.waitKey(0)
    return 0


def OTSU_Seg(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    max_std = 0.0
    ret = 0.0
    t = int(img.shape[0] // 4)
    for i in range(t):
        area = img_gray[i:img.shape[0] - i, i:img.shape[1] - i]
        for k in range(1, area.max()):
            # 像素值小于等于k的像素的均值及比例
            m1 = area[area <= k].mean()
            p1 = area[area <= k].size / area.size

            # 像素值大于k的像素的均值及比例
            m2 = area[area > k].mean()
            p2 = area[area > k].size / area.size

            # 整体的均值及方差（类间方差）
            m = p1 * m1 + p2 * m2
            std = p1 * ((m1 - m) ** 2) + p2 * ((m2 - m) ** 2)

            # 取出使类间方差最大的k
            if std > max_std:
                max_std = std
                ret = k

    ret, ret_img = cv2.threshold(img_gray, ret, 255, cv2.THRESH_BINARY_INV)

    return ret_img


if __name__ == '__main__':
    path = "defect_area_data"
    for file in os.listdir(path):
        raw_img = cv2.imread(os.path.join(path, file))
        bin_img = OTSU_Seg(raw_img)
        get_mask(bin_img, file)
        cv2.waitKey(0)
