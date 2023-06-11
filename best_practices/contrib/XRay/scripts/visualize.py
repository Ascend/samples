from __future__ import print_function
import argparse
import os
import sys
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from PIL import Image
import cv2
import numpy as np
from scipy import misc
from tqdm import tqdm
from torchvision import transforms

cur_path = os.path.abspath(os.path.dirname(__file__))
root_path = os.path.split(cur_path)[0]
sys.path.append(root_path)

from core.models.unet import UNet
from core.utils.distributed import synchronize, get_rank, make_data_sampler, make_batch_data_sampler


def parse_args():
    parser = argparse.ArgumentParser(description='Semantic Segmentation Testing With Pytorch')
    # model and dataset
    parser.add_argument('--model', type=str, default='unet', help='model name (default: unet)')
    parser.add_argument('--dataset', type=str, default='pascal_voc', help='dataset name (default: pascal_voc)')
    parser.add_argument('--data_dir', type=str, default='./data/20221011_casting_voc',
                        help='dataset dir')
    parser.add_argument('--test_dir', type=str, default='/data/ktjiang/X-ray image/26 of 26 X-ray image/8677_jpg', 
                        help='test img dirs without_gt')
    parser.add_argument('--crop_size', type=int, default=512, help='crop image size')
    # testing hyper params
    parser.add_argument('--num_class', type=int, default=4, help='class number (default: 4)')
    # cuda setting
    parser.add_argument('--no-cuda', action='store_true', default=False, help='disables CUDA training')
    parser.add_argument('--local_rank', type=int, default=0)
    parser.add_argument('--device', type=int, default=0, help='npu device')
    # weights
    parser.add_argument('--weights_dir', default='./best_model_miou.pth', 
                        help='Directory for models weights')
    parser.add_argument('--epochs', type=int, default=2000, metavar='N', help='number of epochs to train (default: 50)')
    parser.add_argument('--experiment_name', type=str, default='del', help='experiment name')
    args = parser.parse_args()

    return args


class Evaluator(object):
    """testing"""
    def __init__(self, args):
        """test initialize"""
        self.args = args
        self.device = torch.device(args.device)

        # image transform
        self.input_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.5, .5, .5], [.5, .5, .5]),  # [.485, .456, .406], [.229, .224, .225]
        ])
        # read data
        self.dataset_dir = args.data_dir
        self.result_dir = os.path.join('../results', args.experiment_name)
        self.img_dir = os.path.join(self.dataset_dir, 'JPEGImages')
        self.mask_dir = os.path.join(self.dataset_dir, 'SegmentationClass')
        self.test_img_lists = os.path.join(self.dataset_dir, 'ImageSets/Segmentation/test.txt')
        # create vis dir
        self.res_dir = os.path.join(self.result_dir, 'test_results')  # softmax res
        self.vis_dir = os.path.join(self.result_dir, 'vis_results')  # vis pred with gt
        self.vis_pred_dir = os.path.join(self.result_dir, 'vis_pred')  # vis pred no gt
        self.res_multi_dir = os.path.join(self.result_dir, 'test_results_multi')  # multi class res
        os.makedirs(self.res_dir, exist_ok=True)
        os.makedirs(self.vis_dir, exist_ok=True)
        os.makedirs(self.vis_pred_dir, exist_ok=True)
        os.makedirs(self.res_multi_dir, exist_ok=True)

        self.num_class = args.num_class
        self.patch_size = args.crop_size
        # load weights
        self.model_weights = args.weights_dir
        
	# create network
        self.model = UNet(3, self.num_class, bilinear=True)
        self.model.load_state_dict(torch.load(self.model_weights))
        if args.distributed:
            self.model = nn.parallel.DistributedDataParallel(self.model,
                device_ids=[args.local_rank], output_device=args.local_rank)
        self.model.to(self.device)

    def to_one_hot(self, tensor, n_classes):
        """convert tensor to one hot"""
        n, h, w = tensor.size()
        zero = torch.zeros(n, n_classes, h, w)
        one_hot = zero.scatter_(1, tensor.view(n, 1, h, w), 1)
        return one_hot

    def eval_patch(self, patch):
        """eval each patch"""
        inpu = Image.fromarray(cv2.cvtColor(patch, cv2.COLOR_BGR2RGB))
        inpu = self.input_transform(inpu).unsqueeze(0)
        inpu = inpu.to(self.device)
        with torch.no_grad():
            output, sigmoid = self.model(inpu)
        return output, sigmoid.detach().cpu()  # .numpy().squeeze()

    def vis(self, img, mask, pred):
        """vis results in multi modal (pred and gt)"""
        mask[mask > 0] = 255
        pred = (pred * 255).astype(np.uint8)
        modal = np.zeros_like(img)
        modal[:, :, 1] = pred
        modal[:, :, 2] = mask
        result = cv2.addWeighted(img, 0.6, modal, 0.4, 0)
        return result

    def vis_pred(self, img, pred):
        """vis results only for pred"""
        pred = (pred * 255).astype(np.uint8)
        modal = np.zeros_like(img)
        modal[:, :, 2] = pred
        result = cv2.addWeighted(img, 0.7, modal, 0.3, 0)
        return result

    def eval(self):
        """eval procedure"""
        self.model.eval()
        print('----eval------')
        with open(self.test_img_lists) as f:
            img_names = f.readlines()
            total_imgs = len(img_names)
            for idx, img_name in tqdm(enumerate(img_names)):
                img_name = img_name.split('.')[0]
                img = cv2.imread(os.path.join(self.img_dir, img_name + '.bmp'))
                mask = cv2.imread(os.path.join(self.mask_dir, img_name + '_mask.png'), 0)
                h, w, c = img.shape
                pred = np.zeros((h, w), dtype=np.float32)
                pred_softmax = np.zeros((self.args.num_class, h, w), dtype=np.float32)

                for i in range(0, h, self.patch_size):
                    for j in range(0, w, self.patch_size):
                        patch = img[i: i + self.patch_size, j: j + self.patch_size, :]
                        output, pred_patch = self.eval_patch(patch)
                        pred_softmax[:, i: i + self.patch_size, j: j + self.patch_size] = pred_patch[0]
                        pred_patch[pred_patch < 0.7] = 0
                        pred_patch = torch.argmax(pred_patch, 1)[0].data.numpy()
                        pred_patch = np.array(pred_patch)
                        pred[i: i + self.patch_size, j: j + self.patch_size] = pred_patch
                np.save(os.path.join(self.result_dir, img_name+'.npy'), pred_softmax)
                print('{} / {}'.format(idx+1, total_imgs))

                # multi class
                multi_class = True
                if multi_class:
                    pred = torch.from_numpy(pred).unsqueeze(0)
                    pred = pred.long()
                    one_hot = self.to_one_hot(pred, self.args.num_class).squeeze(0)
                    pred_valid = one_hot[1:, :, :].numpy().transpose(1, 2, 0)
                    for i in range(self.args.num_class-1):
                        cv2.imwrite(os.path.join(self.res_multi_dir, img_name + f'_{i+1}cls.png'), 
                                    pred_valid[..., i] * 255)

                # 2 class
                multi_class = False
                if not multi_class:
                    pred[pred > 0] = 1
                    pred = pred[0].data.numpy()
                    cv2.imwrite(os.path.join(self.res_dir, img_name + '.png'), pred * 255)
                    vis = self.vis(img, mask, pred)
                    vis_pred = self.vis_pred(img, pred)
                    cv2.imwrite(os.path.join(self.vis_dir, img_name + '.png'), vis)
                    cv2.imwrite(os.path.join(self.vis_pred_dir, img_name + '_pred.png'), vis_pred)


    def eval_no_gt(self):
        """eval procedure"""
        self.model.eval()

        for filename in tqdm(os.listdir(args.test_dir)):
            img_name = filename.split('.')[0]
            img_path = os.path.join(args.test_dir, filename)
            img = cv2.imread(img_path)
            h, w, c = img.shape
            pred = np.zeros((h, w), dtype=np.float32)
            pred_softmax = np.zeros((4, h, w), dtype=np.float32)

            for i in range(0, h, self.patch_size):
                for j in range(0, w, self.patch_size):
                    patch = img[i: i + self.patch_size, j: j + self.patch_size, :]
                    output, pred_patch = self.eval_patch(patch)
                    pred_softmax[:, i: i + self.patch_size, j: j + self.patch_size] = pred_patch[0]

                    pred_patch = torch.argmax(pred_patch, 1)[0].data.numpy()
                    pred_patch = np.array(pred_patch)
                    pred[i: i + self.patch_size, j: j + self.patch_size] = pred_patch
            np.save(os.path.join(self.result_dir, img_name + '.npy'), pred_softmax)
            pred[pred > 0] = 1
            cv2.imwrite(os.path.join(self.res_dir, img_name + '.png'), pred * 255)
            vis_pred = self.vis_pred(img, pred)
            cv2.imwrite(os.path.join(self.vis_pred_dir, img_name + '_pred.png'), vis_pred)


if __name__ == '__main__':
    args = parse_args()
    NUM_GPUS =  1
    args.distributed = NUM_GPUS > 1

    args.device = f"npu:{args.device}"
    torch.npu.set_device(args.device)
    args.distributed = False

    if args.distributed:
        torch.cuda.set_device(args.local_rank)
        torch.distributed.init_process_group(backend="nccl", init_method="env://")
        synchronize()

    evaluator = Evaluator(args)
    evaluator.eval()
    torch.cuda.empty_cache()
