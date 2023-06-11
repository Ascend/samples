"""Pascal VOC Semantic Segmentation Dataset."""
import os
import random
import torch
import numpy as np

from PIL import Image
from .segbase import SegmentationDataset


class VOCSegmentation(SegmentationDataset):
    """Pascal VOC Semantic Segmentation Dataset.

    Parameters
    ----------
    root : string
        Path to VOCdevkit folder. Default is './datasets/VOCdevkit'
    split: string
        'train', 'val' or 'test'
    transform : callable, optional
        A function that transforms the image
    Examples
    --------
    >>> from torchvision import transforms
    >>> import torch.utils.data as data
    >>> # Transforms for Normalization
    >>> input_transform = transforms.Compose([
    >>>     transforms.ToTensor(),
    >>>     transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
    >>> ])ia.seed(1)
        # 将图片转换为SegmentationMapOnImage类型
        # segmap = ia.SegmentationMapOnImage(mask, shape=mask.shape, nb_classes=1 + 2)
        sometimes = lambda aug: iaa.Sometimes(0.5, aug)  # 建立lambda表达式，  sometimes表示随机对一部分图像做数据增强操作
        # 定义数据增强方法
        seq = iaa.Sequential([

            sometimes(iaa.Affine(rotate=(90, 90))),  # rotate by -45 to 45 degrees (affects heatmaps)  只随机对一部分图像进行旋转
            iaa.Fliplr(0.2),  # 对20%的图像进行镜像翻转
            iaa.Flipud(0.2),  # 对20%的图像做左右翻转

        ], random_order=True)

        images_aug = []
        segmaps_aug = []

        # 这里可以通过加入循环的方式，对多张图进行数据增强。
        seq_det = seq.to_deterministic()  # 确定一个数据增强的序列
        images_aug = seq_det.augment_image(image)  # 将方法应用在原图像上
        segmaps_aug = seq_det.augment_segmentation_maps([mask])[0].get_arr_int().astype(np.uint8)

        return images_aug, segmaps_aug
    >>> # Create Dataset
    >>> trainset = VOCSegmentation(split='train', transform=input_transform)
    >>> # Create Training Loader
    >>> train_data = data.DataLoader(
    >>>     trainset, 4, shuffle=True,
    >>>     num_workers=4)
    """

    NUM_CLASS = 4  # 21 4

    def __init__(self, root=None, split='train', mode=None, transform=None, base_size=512, crop_size=512, **kwargs):
        super(VOCSegmentation, self).__init__(root, split, mode, transform, base_size, crop_size, **kwargs)
        _mask_dir = os.path.join(root, 'SegmentationClass')
        _image_dir = os.path.join(root, 'JPEGImages')
        # train/val/test splits are pre-cut
        _splits_dir = os.path.join(root, 'ImageSets/Segmentation')
        if split == 'train':
            _split_f = os.path.join(_splits_dir, 'train.txt')
        elif split == 'val':
            _split_f = os.path.join(_splits_dir, 'val.txt')
        elif split == 'test':
            _split_f = os.path.join(_splits_dir, 'test.txt')
        else:
            raise RuntimeError('Unknown dataset split.')

        self.images = []
        self.masks = []
        with open(os.path.join(_split_f), "r") as lines:
            for line in lines:
                _image = os.path.join(_image_dir, line.rstrip('\n'))
                if not os.path.isfile(_image):
                    print(_image)
                assert os.path.isfile(_image)
                self.images.append(_image)
                if split != 'test':
                    _mask = os.path.join(_mask_dir, line.rstrip('.bmp\n') + "_mask.png")
                    assert os.path.isfile(_mask)
                    self.masks.append(_mask)

        if split != 'test':
            assert (len(self.images) == len(self.masks))
        print('Found {} images in the folder {}'.format(len(self.images), root))


    def __getitem__(self, index):
        img = Image.open(self.images[index]).convert('RGB')
        mask = Image.open(self.masks[index]).convert('L')

        # 自定义数据增强
        if self.mode == 'train':
            img, mask = self.aug_mask_image(img, mask)

        img = self._img_transform(img)
        mask = np.array(mask).astype('int32')

        mask[(mask == 1) | (mask == 2)] = 0
        mask[mask != 0 ] -= 2

        h, w = mask.shape
        method = 'patch_stride'
        size = self.crop_size
        # method one
        if method == 'patch':
            idx = h // size
            row = np.random.randint(0, idx)
            col = np.random.randint(0, idx)
            patch = img[size*row:size*(row + 1), size*col:size*(col + 1), :]
            patch_mask = mask[size*row:size*(row + 1), size*col:size*(col + 1)]

            while sum(patch_mask[patch_mask > 0]) == 0:
                row = np.random.randint(0, idx)
                col = np.random.randint(0, idx)
                patch = img[size * row:size * (row + 1), size * col:size * (col + 1), :]
                patch_mask = mask[size * row:size * (row + 1), size * col:size * (col + 1)]
            img = patch
            mask = patch_mask
        # method two
        else:
            row = np.random.randint(0, h - size)
            col = np.random.randint(0, w - size)
            patch = img[row:(row + size), col:(col + size), :]
            patch_mask = mask[row:(row + size), col:(col + size)]

            while sum(patch_mask[patch_mask > 0]) == 0:
                row = np.random.randint(0, h - size)
                col = np.random.randint(0, w - size)
                patch = img[row:(row + size), col:(col + size), :]
                patch_mask = mask[row:(row + size), col:(col + size)]
            if patch_mask.shape != (512, 512) or patch.shape != (512, 512, 3):
                print("asdasdasdasdasd!!!!")
            img = patch
            mask = patch_mask

        if self.mode == 'test':
            img = self._img_transform(img)
            if self.transform is not None:
                img = self.transform(img)
            return img, os.path.basename(self.images[index])

        # synchronized transform
        if self.mode == 'train':
            pass
        elif self.mode == 'val':
            pass
        else:
            assert self.mode == 'testval'
            img, mask = self._img_transform(img), self._mask_transform(mask)

        # general resize, normalize and toTensor
        if self.transform is not None:
            img = self.transform(img)

        mask[mask == 255] = -1
        mask = torch.from_numpy(mask)
        return patch, img, mask, os.path.basename(self.images[index])


    def __len__(self):
        return len(self.images)


    @property
    def classes(self):
        """Category names."""
        return ('background', 'lowdensity', 'highdensity', 'qikong')  # (,'defect')


    def aug_mask_image(self, image, mask):
        rd = random.random()
        if rd > 0.8:
            image = image.rotate(90)
            mask = mask.rotate(90)
        rd = random.random()
        if rd > 0.8 :
            image = image.transpose(Image.FLIP_LEFT_RIGHT)  # 水平翻转
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)  # 水平翻转
        rd = random.random()
        if rd > 0.8:
            image = image.transpose(Image.FLIP_TOP_BOTTOM)  # 垂直翻转
            mask = mask.transpose(Image.FLIP_TOP_BOTTOM)  # 垂直翻转
        return image, mask


    def _mask_transform(self, mask):
        target = np.array(mask).astype('int32')
        target[target == 255] = -1
        return torch.from_numpy(target)



if __name__ == '__main__':
    dataset = VOCSegmentation()