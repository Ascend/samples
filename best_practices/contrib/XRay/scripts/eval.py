import argparse
import time
import datetime
import os
import shutil
import sys
import numpy as np
import cv2
from tqdm import tqdm
from tensorboardX import SummaryWriter
from PIL import Image
import torch
import torch.nn as nn
import torch.utils.data as data
import torch.backends.cudnn as cudnn
from torchvision import transforms

cur_path = os.path.abspath(os.path.dirname(__file__))
root_path = os.path.split(cur_path)[0]
sys.path.append(root_path)

from core.data.dataloader.pascal_voc import VOCSegmentation
from core.models.unet import UNet
from core.utils.loss import l2loss, MulticlassDiceLoss, SoftIoULoss, DiceLoss
from core.utils.distributed import *
from core.utils.logger import setup_logger
from core.utils.lr_scheduler import WarmupPolyLR
from core.utils.score import SegmentationMetric


def parse_args():
    parser = argparse.ArgumentParser(description='Semantic Segmentation Training With Pytorch')
    # model and dataset
    parser.add_argument('--model', type=str, default='unet', help='model name (default: unet)')
    parser.add_argument('--dataset', type=str, default='pascal_voc', help='dataset name (default: pascal_voc)')
    parser.add_argument('--data_dir', type=str, default='./data/20221011_casting_voc', help='dataset dir')
    parser.add_argument('--base_size', type=int, default=512, help='crop image size')  # 480
    parser.add_argument('--crop_size', type=int, default=512, help='crop image size')  # 480
    parser.add_argument('--workers', '-j', type=int, default=8, metavar='N', help='dataloader threads')
    parser.add_argument('--batch_size', type=int, default=2, metavar='N', help='batch size(default: 8)')
    # training hyper params
    parser.add_argument('--jpu', action='store_true', default=False, help='JPU')
    parser.add_argument('--num_class', type=int, default=4, help='class number (default: 4)')
    parser.add_argument('--no-npu', action='store_true', default=False, help='disables npu training')
    parser.add_argument('--local_rank', type=int, default=0)
    parser.add_argument('--device', type=int, default=0, help='npu device')
    # checkpoint and log
    parser.add_argument('--weight', type=str, default='./checkpoints/best_model_miou.pth', 
                        help='put the path to resuming file if needed')
    parser.add_argument('--checkpoint-dir', default='../checkpoints/',
                        help='Directory for saving checkpoint models')
    parser.add_argument('--save-epoch', type=int, default=100, help='save model every checkpoint-epoch')
    parser.add_argument('--log-dir', default='../runs/logs', help='Directory for saving checkpoint models')

    args = parser.parse_args()

    return args


class Evaler(object):
    """training"""
    def __init__(self, args):
        """training initialize"""
        self.args = args
        self.device = torch.device(args.device)

        # image transform
        self.input_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.5, .5, .5], [.5, .5, .5]),
        ])

        self.dataset_dir = args.data_dir
        self.img_dir = os.path.join(self.dataset_dir, 'JPEGImages')
        self.mask_dir = os.path.join(self.dataset_dir, 'SegmentationClass')
        self.test_img_lists = os.path.join(self.dataset_dir, 'ImageSets/Segmentation/test.txt')

        self.patch_size = args.crop_size

        # create network
        self.model = UNet(3, args.num_class, bilinear=True).npu()
        # resume checkpoint if needed
        if args.weight:
            if os.path.isfile(args.weight):
                name, ext = os.path.splitext(args.weight)
                assert ext == '.pkl' or '.pth', 'Sorry only .pth and .pkl files supported.'
                print('model weight, loading {}...'.format(args.weight))
                self.model.load_state_dict(torch.load(args.weight, map_location=lambda storage, loc: storage))


        # optimizer, for model just includes pretrained, head and auxlayer
        params_list = list()
        params_list.append({'params': self.model.parameters()})


        if args.distributed:
            self.model = nn.parallel.DistributedDataParallel(self.model, device_ids=[args.local_rank],
                                                             output_device=args.local_rank)

        # evaluation metrics
        self.best_pred = float('inf')
        self.best_miou = 0
        self.pixacc = None

        self.metric = SegmentationMetric(args.num_class)

    def eval_patch(self, patch):
        """eval each patch"""
        inpu = Image.fromarray(cv2.cvtColor(patch, cv2.COLOR_BGR2RGB))
        inpu = self.input_transform(inpu).unsqueeze(0)
        inpu = inpu.to(self.device)
        with torch.no_grad():
            output, sigmoid = self.model(inpu)
        return output, sigmoid.detach().cpu()

    def validation(self):
        """validation procedure"""
        is_best = False
        is_best_miou = False
        torch.npu.empty_cache()  
        self.model.eval()
        self.metric.reset()

        pix_accs, mious = [], []

        with open(self.test_img_lists) as f:
            img_names = f.readlines()
            total_imgs = len(img_names)
            start = time.time()
            for idx, img_name in tqdm(enumerate(img_names)):
                img_name = img_name.split('.')[0]
                img = cv2.imread(os.path.join(self.img_dir, img_name + '.bmp'))
                mask = cv2.imread(os.path.join(self.mask_dir, img_name + '_mask.png'), 0)
                # 4 cls
                mask[(mask == 1) | (mask == 2)] = 0
                mask[mask != 0] -= 2
                mask[mask == 255] = -1

                mask = torch.from_numpy(mask)

                h, w, c = img.shape
                pred = np.zeros((h, w), dtype=np.float32)
                pred_softmax = np.zeros((self.args.num_class, h, w), dtype=np.float32)

                for i in range(0, h, self.patch_size):
                    for j in range(0, w, self.patch_size):
                        patch = img[i: i + self.patch_size, j: j + self.patch_size, :]
                        output, pred_patch = self.eval_patch(patch)
                        target = mask[i: i + self.patch_size, j: j + self.patch_size]
                        self.metric.update(output, target.to(output.device))
                        pix_acc, miou = self.metric.get()
                        pix_accs.append(pix_acc)
                        mious.append(miou)
            end = time.time()
        logger.info("Infer time is {:.3f}s per img".format((end - start)/total_imgs))

        logger.info("validation pix_acc: {:.3f}, miou: {:.3f}".format(
            np.mean(pix_accs) * 100, np.mean(mious) * 100))



if __name__ == '__main__':
    args = parse_args()

    num_gpus = 1
    args.num_gpus = num_gpus
    args.distributed = num_gpus > 1
    args.device = f"npu:{args.device}"
    torch.npu.set_device(args.device)

    logger = setup_logger("semantic_segmentation", args.log_dir+'_', get_rank(),
                          filename='{}_log.txt'.format(args.model))
    logger.info("Using {} NPUs".format(num_gpus))
    logger.info(args)

    evaler = Evaler(args)
    evaler.validation()
    torch.npu.empty_cache()
