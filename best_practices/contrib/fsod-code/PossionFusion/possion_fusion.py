import cv2
import random
import numpy as np
from PossionFusion.utils import filp_image, resize_image, rotate_image, get_edge_cor
import os
from PossionFusion.args import get_PF_args


def seam_clone(defect_img, des_img, output, carry, src_mask, t, c):
    kernel = np.ones((3, 3))
    src_mask = cv2.dilate(src_mask, kernel, iterations=12)
    defect_img, src_mask = filp_image(defect_img, src_mask)
    defect_img, src_mask = resize_image(defect_img, src_mask)
    defect_img, src_mask = rotate_image(defect_img, src_mask)
    blur_defect_img = cv2.GaussianBlur(defect_img, (3, 3), 0.8, 0.8)
    if carry and (output is not None):
        jx_img = output
    else:
        jx_img = des_img
    row1, row2 = get_edge_cor(jx_img)

    row = (row1 + row2) // 2
    W = jx_img.shape[1]
    H = row2 - row1
    col = W // 2

    # 计算偏移量
    if t == 1:
        y_offset = random.randint(-H // 4, H // 4)
        x_offset = random.randint(-W // 4, W // 4)
    else:
        if c == 0:
            y_offset = random.randint(H // 8, H // 4)
            x_offset = random.randint(W // 8, W // 4)
        else:
            y_offset = -random.randint(H // 8, H // 4)
            x_offset = -random.randint(W // 8, W // 4)
    # 获得融合中心位置
    x = col + x_offset
    y = row + y_offset
    # 判断融合结果是否会出界
    mask_h, mask_w = src_mask.shape[0:2]
    print(mask_h, mask_w)
    if (y - mask_h // 2) > row1 and (y + mask_h // 2) < row2 and (x - mask_w // 2) > 0 and (x + mask_w // 2) < W - 1:
        center = (x, y)
        # 对缺陷边缘金相泊松融合
        output = cv2.seamlessClone(blur_defect_img, jx_img, src_mask, center, cv2.NORMAL_CLONE)
        output = cv2.GaussianBlur(output, (3, 3), 0.8, 0.8)
        fusion_finish = True
        return fusion_finish, output
    else:
        fusion_finish = False
        return fusion_finish, jx_img


args = get_PF_args()


def main(args=args):
    if not os.path.exists("gen_data"):
        os.mkdir("gen_data")

    des_root_path = args.normal_data
    des_path_list = os.listdir(des_root_path)

    src_root_path = args.defect_data
    src_path_list = os.listdir(src_root_path)

    mask_root_path = args.mask_data
    mask_path_list = os.listdir(mask_root_path)

    for i in range(0, args.times):
        des_index = random.randint(0, len(des_path_list) - 1)
        des_path = os.path.join(des_root_path, des_path_list[des_index])
        des_img = cv2.imread(des_path)
        carry = 0
        output = None
        for j in range(0, args.defect_nums):
            # 获取src_img(缺陷图片)
            src_index = random.randint(0, len(src_path_list) - 1)
            src_path = os.path.join(src_root_path, src_path_list[src_index])
            src_img = cv2.imread(src_path)
            # 获取mask
            if args.method == 1:
                mask = 255 * np.ones(src_img.shape[0:2])
            else:
                mask_path = os.path.join(mask_root_path, mask_path_list[src_index])
                mask = cv2.imread(mask_path)

            fusion_finish, output = seam_clone(src_img, des_img, output, carry, mask, args.defect_nums, j)
            carry = 1
        if fusion_finish:
            cv2.imwrite(f"gen_data/{des_path.split('.')[0][-4:]}_{src_path.split('.')[0][-4:]}_output.png",
                        output)
            print("done")
        else:
            continue


if __name__ == "__main__":
    args = get_PF_args()
    main(args)
    # src_root_path = "data"
    # des_root_path = "normal_data"  # 正常图片存放的地址
    # des_path_list = os.listdir(des_root_path)
    # src_path_list = os.listdir(src_root_path)
    # defect_nums = 1  # 最多融合的缺陷个数，<=2
    # times = 1000  # 融合过程循环次数
    # if not os.path.exists("gen_data"):
    #     os.mkdir("gen_data")
    # for i in range(0, times):
    #     des_index = random.randint(0, len(des_path_list) - 1)
    #     des_path = os.path.join(des_root_path, des_path_list[des_index])
    #     jx_img = cv2.imread(des_path)
    #     carry = 0
    #     output = None
    #     for j in range(0, defect_nums):
    #         src_index = random.randint(0, len(src_path_list) - 1)
    #         src_path = os.path.join(src_root_path, src_path_list[src_index], "img.png")
    #         defect_img = cv2.imread(src_path)
    #         mask_path = os.path.join(src_root_path, src_path_list[src_index], 'mask.png')
    #         mask = cv2.imread(mask_path)
    #         print(mask_path)
    #         fusion_finish, output = seam_clone(defect_img, jx_img, output, carry, mask, defect_nums, j)
    #         carry = 1
    #     if fusion_finish:
    #         cv2.imwrite(f"gen_data/{des_path.split('.')[0][-4:]}_{src_path.split('.')[0][-8:-4]}_output.png",
    #                     output)
    #         print("done")
    #     else:
    #         continue
