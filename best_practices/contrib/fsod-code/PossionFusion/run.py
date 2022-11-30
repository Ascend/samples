from PossionFusion.args import get_PF_args
import os

if __name__ == '__main__':
    args = get_PF_args()
    if args.method == 3 and not args.fusion_only:
        os.system("python get_data.py")
        os.system("python get_split_data.py")
    os.system("python possion_fusion.py")
