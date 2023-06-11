import os
import sys
import argparse
import torch

cur_path = os.path.abspath(os.path.dirname(__file__))
root_path = os.path.split(cur_path)[0]
sys.path.append(root_path)

from core.models.unet import UNet

parser = argparse.ArgumentParser(description="TernaryBert export model")
parser.add_argument("--device_target", type=str, default="Ascend", choices=["Ascend", "GPU"],
                    help="device where the code will be implemented. (Default: Ascend)")
parser.add_argument("--file_name", type=str, default="rilian", help="The name of the output file.")
parser.add_argument("--num_class", type=int, default=4, help="num class")
parser.add_argument("--file_format", type=str, default="MINDIR", choices=["AIR", "MINDIR"],
                    help="output model type")
parser.add_argument("--ckpt_file", type=str, default="", help="pretrained checkpoint file")
parser.add_argument('--crop_size', type=int, default=512, help='crop image size')  # 480
parser.add_argument('--weight', type=str, default='./checkpoints/best_model_miou.pth', \
    help='put the path to resuming file if needed')
parser.add_argument('--output_file', type=str, default='./best_model_miou.onnx', \
    help='put the path to resuming file if needed')

args = parser.parse_args()


def pth2onnx(input_file, output_file):
    model = UNet(3, args.num_class, bilinear=True)
    model.load_state_dict(torch.load(args.weight, map_location=lambda storage, loc: storage))
    model.eval()  
    dummy_input = torch.rand(1, 3, args.crop_size, args.crop_size) 
    dummy_output = model(dummy_input)
    torch.onnx.export(model, dummy_input, output_file, example_outputs=dummy_output, opset_version=11) 

pth2onnx(args.weight, args.output_file)

