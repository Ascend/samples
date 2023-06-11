import argparse
import time
import datetime
import os
import shutil
import sys
import numpy as np
import cv2
from tensorboardX import SummaryWriter
import torch
import torch.nn as nn
import torch.npu
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
    parser.add_argument('--data_dir', type=str, default='', help='dataset dir')
    parser.add_argument('--base_size', type=int, default=512, help='crop image size')  # 480
    parser.add_argument('--crop_size', type=int, default=512, help='crop image size')  # 480
    parser.add_argument('--workers', '-j', type=int, default=8, metavar='N', help='dataloader threads')
    # training hyper params
    parser.add_argument('--jpu', action='store_true', default=False, help='JPU')
    parser.add_argument('--batch_size', type=int, default=2, metavar='N', help='batch size(default: 8)')
    parser.add_argument('--start_epoch', type=int, default=0, metavar='N', help='start epochs (default:0)')
    parser.add_argument('--epochs', type=int, default=100, metavar='N', help='number of epochs to train (default: 50)')
    parser.add_argument('--lr', type=float, default=1e-4, metavar='LR', help='learning rate (default: 1e-4)')
    parser.add_argument('--optim', type=str, default='adam', help='optimizer sgd or adam (default: adam)')
    parser.add_argument('--loss', type=str, default='dice', help='loss, ce, bce or dice or iou (default: ce)')
    parser.add_argument('--num_class', type=int, default=4, help='class number (default: 4)')
    parser.add_argument('--num_gpus', type=int, default=1, help='gpu number (default: 1)')
    parser.add_argument('--momentum', type=float, default=0.9, metavar='M', help='momentum (default: 0.9)')
    parser.add_argument('--weight-decay', type=float, default=1e-4, metavar='M', help='w-decay (default: 5e-4)')
    parser.add_argument('--warmup-iters', type=int, default=0, help='warmup iters')
    parser.add_argument('--warmup-factor', type=float, default=1.0 / 3, help='lr = warmup_factor * lr')
    parser.add_argument('--warmup-method', type=str, default='linear', help='method of warmup')
    # cuda setting
    parser.add_argument('--no-cuda', action='store_true', default=False, help='disables CUDA training')
    parser.add_argument('--local_rank', type=int, default=0)
    parser.add_argument('--device', type=int, default=0, help='npu device')
    # checkpoint and log
    parser.add_argument('--resume', type=str, default=None, help='put the path to resuming file if needed')
    parser.add_argument('--checkpoint-dir', default='../checkpoints/',
                        help='Directory for saving checkpoint models')
    parser.add_argument('--save-epoch', type=int, default=1, help='save model every checkpoint-epoch')
    parser.add_argument('--log-dir', default='../runs/logs', help='Directory for saving checkpoint models')
    parser.add_argument('--log-iter', type=int, default=100, help='print log every log-iter')
    parser.add_argument('--experiment_name', type=str, default='rilian_epoch100', help='experiment name')
    # evaluation only
    parser.add_argument('--val-epoch', type=int, default=1, help='run validation every val-epoch')
    parser.add_argument('--skip-val', action='store_true', default=False, help='skip validation during training')
    args = parser.parse_args()

    return args


class Trainer(object):
    """training"""
    def __init__(self, args):
        """training initialize"""
        self.args = args
        self.device = torch.device(args.device)

        # image transform
        input_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.5, .5, .5], [.5, .5, .5]),  # [.485, .456, .406], [.229, .224, .225]
        ])
        # dataset and dataloaderd

        data_kwargs = {'transform': input_transform, 'base_size': args.base_size, 'crop_size': args.crop_size}
        train_dataset = VOCSegmentation(root=args.data_dir, split='train', mode='train', **data_kwargs)
        val_dataset = VOCSegmentation(root=args.data_dir, split='val', mode='val', **data_kwargs)
        args.iters_per_epoch = len(train_dataset) // (args.num_gpus * args.batch_size)
        args.max_iters = args.epochs * args.iters_per_epoch

        train_sampler = make_data_sampler(train_dataset, shuffle=True, distributed=args.distributed)
        train_batch_sampler = make_batch_data_sampler(train_sampler, args.batch_size, args.max_iters)
        val_sampler = make_data_sampler(val_dataset, False, args.distributed)
        val_batch_sampler = make_batch_data_sampler(val_sampler, args.batch_size)

        self.train_loader = data.DataLoader(dataset=train_dataset,
                                            batch_sampler=train_batch_sampler,
                                            num_workers=args.workers,
                                            pin_memory=True)
        self.val_loader = data.DataLoader(dataset=val_dataset,
                                          batch_sampler=val_batch_sampler,
                                          num_workers=args.workers,
                                          pin_memory=True)

        # create network
        self.model = UNet(3, args.num_class, bilinear=True).npu()
        # resume checkpoint if needed
        if args.resume:
            if os.path.isfile(args.resume):
                name, ext = os.path.splitext(args.resume)
                assert ext == '.pkl' or '.pth', 'Sorry only .pth and .pkl files supported.'
                print('Resuming training, loading {}...'.format(args.resume))
                self.model.load_state_dict(torch.load(args.resume, map_location=lambda storage, loc: storage))

        # create criterion
        if args.loss == 'bce':
            self.criterion = nn.BCEWithLogitsLoss()
        elif args.loss == 'ce':
            self.criterion = nn.CrossEntropyLoss()
        elif args.loss == 'dice':
            self.criterion = MulticlassDiceLoss()
        elif args.loss == 'iou':
            self.criterion = SoftIoULoss(n_classes=args.num_class)

        # optimizer, for model just includes pretrained, head and auxlayer
        params_list = list()
        params_list.append({'params': self.model.parameters(), 'lr': args.lr})
        if args.optim == 'sgd':
            self.optimizer = torch.optim.SGD(params_list,
                                             lr=args.lr,
                                             momentum=args.momentum,
                                             weight_decay=args.weight_decay)
        elif args.optim == 'adam':
            self.optimizer = torch.optim.Adam(params_list,
                                              lr=args.lr,
                                              weight_decay=args.weight_decay)

        # lr scheduling
        self.lr_scheduler = WarmupPolyLR(self.optimizer,
                                         max_iters=args.max_iters,
                                         power=0.9,
                                         warmup_factor=args.warmup_factor,
                                         warmup_iters=args.warmup_iters,
                                         warmup_method=args.warmup_method)

        if args.distributed:
            self.model = nn.parallel.DistributedDataParallel(self.model, device_ids=[args.local_rank],
                                                             output_device=args.local_rank)

        # evaluation metrics
        self.best_pred = float('inf')
        self.best_miou = 0
        self.pixacc = None

        # tensorboard vis loss
        self.writer = SummaryWriter('../tensorboard/' + args.experiment_name)

        self.metric = SegmentationMetric(val_dataset.num_class)

        self.pixacc_list = []
        self.miou_list = []
        self.valloss_list = []
        self.trainloss_list = []

    def train(self):
        """training procedure"""
        save_to_disk = get_rank() == 0
        epochs, max_iters = self.args.epochs, self.args.max_iters
        log_per_iters, val_per_iters = self.args.log_iter, self.args.val_epoch * self.args.iters_per_epoch
        save_per_iters = self.args.save_epoch * self.args.iters_per_epoch
        start_time = time.time()
        logger.info('Start training, Total Epochs: {:d} = Total Iterations {:d}'.format(epochs, max_iters))

        self.model.train()
        for iteration, (patch, images, targets, _) in enumerate(self.train_loader):
            iteration = iteration + 1
            self.lr_scheduler.step()

            images = images.to(self.device)
            targets = targets.to(self.device)
            outputs, sigmoid = self.model(images)

            losses = self.criterion(outputs, targets )

            self.optimizer.zero_grad()
            losses.backward()
            self.optimizer.step()

            eta_seconds = ((time.time() - start_time) / iteration) * (max_iters - iteration)
            eta_string = str(datetime.timedelta(seconds=int(eta_seconds)))
            self.writer.add_scalar('train/loss', losses.item(), iteration)
            if iteration % log_per_iters == 0 and save_to_disk:
                logger.info(
                    "Iters: {:d}/{:d} || Lr: {:.6f} || Loss: {:.4f} || Cost Time: {} || Estimated Time: {}".format(
                        iteration, max_iters, self.optimizer.param_groups[0]['lr'], losses.item(),
                        str(datetime.timedelta(seconds=int(time.time() - start_time))), eta_string))
                epoch = int(iteration // log_per_iters)
                vis(patch, targets, sigmoid, epoch, self.args)
            if iteration % save_per_iters == 0 and save_to_disk:
                epoch = int(iteration // args.iters_per_epoch)
                save_checkpoint(self.model, self.args, epoch, is_best=False)

            if not self.args.skip_val and iteration % val_per_iters == 0:
                epoch = int(iteration // args.iters_per_epoch)
                self.trainloss_list.append(losses.item())
                self.validation(epoch)
                self.model.train()


        total_training_time = time.time() - start_time
        total_training_str = str(datetime.timedelta(seconds=total_training_time))
        logger.info(
            "Total training time: {} ({:.4f}s / it)".format(
                total_training_str, total_training_time / max_iters))
        self.writer.close()
        self.draw_result()
        logger.info("best miou is {:.4f}, pixacc is {:.4f})".format(self.best_miou, self.pixacc))

    def draw_result(self):
        y1 = self.trainloss_list
        y2 = self.valloss_list
        y3 = self.miou_list
        y4 = self.pixacc_list

        import matplotlib.pyplot as plt
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False

        x = range(len(y1))
        plt.plot(x, y1, 'g-', mec='k', label='train Loss', lw=2)
        plt.plot(x, y2, 'b-', mec='k', label='val Loss', lw=2)
        plt.plot(x, y3, 'm-', mec='k', label='miou', lw=2)
        plt.plot(x, y4, 'r-', mec='k', label='pixacc', lw=2)
        plt.legend(loc='upper right', fontsize=15)

        plt.xlabel(u"epoch")

        plt.savefig(os.path.join(self.args.checkpoint_dir, self.args.experiment_name, 'result.jpg'))

    def validation(self, epoch):
        """validation procedure"""
        is_best = False
        is_best_miou = False
        if self.args.distributed:
            model = self.model.module
        else:
            model = self.model
        torch.npu.empty_cache()
        model.eval()
        self.metric.reset()

        pix_accs, mious = [], []
        for i, (patch, image, target, filename) in enumerate(self.val_loader):
            image = image.to(self.device)
            target = target.to(self.device)

            with torch.no_grad():
                outputs, sigmoid = self.model(image)
                losses = self.criterion(outputs, target )

            self.metric.update(outputs, target)
            pix_acc, miou = self.metric.get()
            pix_accs.append(pix_acc)
            mious.append(miou)

        logger.info("Epoch: {:d}, validation pix_acc: {:.3f}, mIoU: {:.3f}, Loss: {:.4f}".format(
            epoch, np.mean(pix_accs) * 100, np.mean(mious) * 100, losses))

        self.valloss_list.append(losses.item())
        self.miou_list.append(np.mean(mious) * 100)
        self.pixacc_list.append(np.mean(pix_accs) * 100)

        self.writer.add_scalar('val/loss', losses.item(), epoch)
        if losses < self.best_pred:
            is_best = True
            self.best_pred = losses
        if np.mean(mious) > self.best_miou:
            is_best_miou = True
            self.best_miou = np.mean(mious)
            self.pixacc = np.mean(pix_accs)
        if is_best:
            save_checkpoint(self.model, self.args, epoch, is_best=is_best)
        if is_best_miou:
            save_checkpoint(self.model, self.args, epoch, is_best_miou=is_best_miou)
        synchronize()


def save_checkpoint(model, args, epoch, is_best=False, is_best_miou=False):
    """Save Checkpoint"""
    directory = os.path.expanduser(args.checkpoint_dir+args.experiment_name)
    if not os.path.exists(directory):
        os.makedirs(directory)
    filename = '{}.pth'.format(epoch)
    filename = os.path.join(directory, filename)

    if args.distributed:
        model = model.module
    torch.save(model.state_dict(), filename)
    if is_best:
        best_filename = 'best_model.pth'
        best_filename = os.path.join(directory, best_filename)
        shutil.copyfile(filename, best_filename)
    if is_best_miou:
        best_filename = 'best_model_miou.pth'
        best_filename = os.path.join(directory, best_filename)
        shutil.copyfile(filename, best_filename)


def vis(img, target, output, epoch, args):
    """ visualize images during training"""
    save_dir = '../visualize/vis_' + args.experiment_name
    os.makedirs(save_dir, exist_ok=True)

    input_img = img[0].numpy()
    input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
    input_img = np.clip(input_img, 0, 255)
    # multi class
    pred = torch.argmax(output, dim=1)[0].cpu().data.numpy()

    pred = np.array(pred)

    # multi class
    pred[pred > 0] = 1
    pred *= 255

    target = target[0].detach().cpu().numpy()
    target[target > 0] = 255

    vis_img = np.hstack((input_img, target, pred))

    name = 'vis_' + str(epoch) + '.jpg'
    cv2.imwrite(os.path.join(save_dir, name), vis_img.astype(np.uint8))


if __name__ == '__main__':
    args = parse_args()

    # reference maskrcnn-benchmark
    args.device = f"npu:{args.device}"
    torch.npu.set_device(args.device)
    args.distributed = False
    num_gpus = 1
    if args.distributed:
        torch.cuda.set_device(args.local_rank)
        torch.distributed.init_process_group(backend="nccl", init_method="env://")
        synchronize()
    args.lr = args.lr * num_gpus

    logger = setup_logger("semantic_segmentation", args.log_dir+'_'+args.experiment_name, get_rank(),
                          filename='{}_log.txt'.format(args.model))
    logger.info("Using {} NPUs".format(num_gpus))
    logger.info(args)

    trainer = Trainer(args)
    trainer.train()
    torch.npu.empty_cache()