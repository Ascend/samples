"""
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""


import os
import argparse
import socket

import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP
from torchvision import datasets
from torchvision import transforms
from amct_pytorch.tensor_decompose import auto_decomposition
from amct_pytorch.tensor_decompose import decompose_network

from common.model import Net
from common.train import train
from common.test import test


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='PyTorch MNIST Example')
    parser.add_argument('--backend', type=str, default='gloo',
                        choices=['gloo', 'nccl'], help='Backend of DDP. '
                        'gloo for CPU, nccl for GPU (default: gloo)')
    parser.add_argument('--init_method', type=str, default='env://',
                        help='Initialization method of DDP (default: env://)')
    parser.add_argument('--local_rank', type=int, default=0,
                        help='Local rank of DDP (default: 0)')
    parser.add_argument('--data-path', type=str, default='data',
                        help='Path to MNIST data (default: data)')
    parser.add_argument('--train-batch-size', type=int, default=32,
                        help='Batch size on each device for training '
                        '(default: 32)')
    parser.add_argument('--test-batch-size', type=int, default=1000,
                        help='Batch size on the first device for testing '
                        '(default: 1000)')
    parser.add_argument('--lr', type=float, default=0.01,
                        help='Learning rate (default: 0.01)')
    parser.add_argument('--steps', type=int, default=200,
                        help='Number of training iterations (default: 200)')
    parser.add_argument('--use-gpu', action='store_true',
                        help='Whether to use GPU training')
    parser.add_argument('--seed', type=int, default=1,
                        help='Random seed (default: 1)')
    parser.add_argument('--log-steps', type=int, default=20,
                        help='Iterations interval for logging (default: 20)')
    parser.add_argument('--save-path', type=str, default=None,
                        help='Path to save model weights after training '
                        '(default: None)')
    parser.add_argument('--pretrained-path', type=str, default=None,
                        help='Path to the pretrained weights. Required in '
                        'tensor decomposition online mode (default: None)')
    parser.add_argument('--tensor-decompose', action='store_true',
                        help='Whether to use tensor decomposition')
    parser.add_argument('--decompose-info-path', type=str, default=None,
                        help='Decomposition information file path, should be '
                        'used along with --tensor-decompose (default: None)')
    parser.add_argument('--decomposed-weights-path', type=str, default=None,
                        help='Path to the decomposed weights, should be used '
                        'along with --tensor-decompose (default: None)')
    parser.add_argument('--run-mode', type=str, default='online',
                        choices=['online', 'offline'], help='Run mode of '
                        'tensor decomposition, should be used along with '
                        '--tensor-decompose (default: online)')
    return parser.parse_args()


def check_args(args):
    """Check input arguments."""
    if args.local_rank < 0:
        raise ValueError('--local_rank should be greater than or equal to 0.')
    if not os.path.exists(args.data_path):
        raise ValueError('--data-path does not exist.')
    for arg, name in [
            (args.train_batch_size, '--train-batch-size'),
            (args.test_batch_size, '--test-batch-size'),
            (args.lr, '--lr'),
            (args.steps, '--steps'),
            (args.log_steps, '--log-steps'),
            ]:
        if arg <= 0:
            raise ValueError('{} should be greater than 0.'.format(name))
    if args.tensor_decompose:
        path_check_list = [
            (args.decompose_info_path, '--decompose-info-path'),
            (args.decomposed_weights_path, '--decomposed-weights-path')]
        for arg, name in path_check_list:
            if arg is None:
                raise ValueError('{} is required when using '
                                 '--tensor-decompose.'.format(name))
        if args.run_mode == 'online':
            arg, name = args.pretrained_path, '--pretrained-path'
            if arg is None:
                raise ValueError('{} is required in online mode.'.format(name))
            if not os.path.isfile(arg):
                ValueError('{} is not a file: {}.'.format(name, arg))
        else:  # offline
            for arg, name in path_check_list:
                if not os.path.isfile(arg):
                    ValueError('{} is not a file: {}.'.format(name, arg))


def main():
    """Main process."""

    # Settings
    args = parse_args()
    check_args(args)
    use_gpu = args.use_gpu and torch.cuda.is_available()
    torch.manual_seed(args.seed)
    local_rank = args.local_rank  # Local identifier of the current node
    device = torch.device('cpu')
    if use_gpu:
        torch.cuda.manual_seed(args.seed)
        torch.backends.cudnn.deterministic = True  # Make the results exactly
        torch.backends.cudnn.benchmark = False  # the same under the same seed
        device = local_rank
        torch.cuda.set_device(device)
    dist.init_process_group(backend=args.backend, init_method=args.init_method)
    rank = dist.get_rank()  # Unique identifier among all processes

    # Prepare data
    socket.setdefaulttimeout(60)  # Connection time limit for data downloading
    transform = transforms.Compose([transforms.ToTensor(), ])
    if local_rank == 0:
        train_dataset = datasets.MNIST(
            args.data_path, train=True, download=True, transform=transform)
    dist.barrier()  # Wait for local_rank 0 to finish downloading the dataset
    if local_rank != 0:
        train_dataset = datasets.MNIST(
            args.data_path, train=True, transform=transform)
    test_dataset = datasets.MNIST(
        args.data_path, train=False, transform=transform)
    train_sampler = torch.utils.data.DistributedSampler(
        train_dataset, shuffle=True)
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=args.train_batch_size, shuffle=False,
        num_workers=1, pin_memory=use_gpu, sampler=train_sampler)
    test_loader = torch.utils.data.DataLoader(
        test_dataset, batch_size=args.test_batch_size, shuffle=False,
        num_workers=1, pin_memory=use_gpu)

    # Build the model
    model = Net()

    # Pretrained weights should be loaded before using auto_decomposition
    if args.pretrained_path is not None:
        pretrained_path = os.path.realpath(args.pretrained_path)
        state_dict = torch.load(pretrained_path, map_location='cpu')
        model.load_state_dict(state_dict)
        if rank == 0:
            print('Loaded pretrained weights file: {}'.format(pretrained_path))

    # Use tensor decomposition here, after building the model, before DDP and
    # before passing model parameters to the optimizer
    if args.tensor_decompose:
        dec_info_path = args.decompose_info_path
        dec_weights_path = os.path.realpath(args.decomposed_weights_path)
        if args.run_mode == 'online':
            if local_rank == 0:
                # Decompose the model on local_rank 0, and save the
                # decomposition information file. It will take some time.
                # Pretrained weights should have been loaded to the model.
                model, _ = auto_decomposition(model, dec_info_path)

                # Save the decomposed weights
                os.makedirs(os.path.dirname(dec_weights_path), exist_ok=True)
                torch.save(model.state_dict(), dec_weights_path)
                print('Decomposed weights file is saved to: {}'.format(
                    dec_weights_path))

            # Wait until all local_ranks get here. This makes the other
            # local_ranks wait for local_rank 0 to complete, in order to use
            # the decomposition information and weights saved by local_rank 0.
            dist.barrier()

            if local_rank != 0:
                # Decompose the model with the saved decomposition information
                model, _ = decompose_network(model, dec_info_path)

                # Load the saved weights
                state_dict = torch.load(dec_weights_path, map_location='cpu')
                model.load_state_dict(state_dict)

        else:  # offline
            # Use existing decomposition information and decomposed weights for
            # all local_ranks
            model, _ = decompose_network(model, dec_info_path)
            state_dict = torch.load(dec_weights_path, map_location='cpu')
            model.load_state_dict(state_dict)
            if rank == 0:
                print('Loaded decomposed weights file: {}'.format(
                    dec_weights_path))

    # Put the model on target device
    model = model.to(device)

    # DDP
    device_ids = [local_rank] if use_gpu else []
    model = DDP(model, device_ids=device_ids)

    # Build optimizer
    # Note: please do not load optimizer parameters of the original model after
    # tensor decomposition, since the model parameters have been changed.
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    # Train and test
    train(model, device, train_loader,
          optimizer, args.steps, args.log_steps, rank == 0)
    if rank == 0:
        test(model, device, test_loader)

    # Save model weights
    if rank == 0 and args.save_path is not None:
        save_path = os.path.realpath(args.save_path)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        torch.save(model.module.state_dict(), save_path)
        print('Trained weights file is saved to: {}'.format(save_path))


if __name__ == '__main__':
    main()
