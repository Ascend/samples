import argparse


def get_PF_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json_dir", type=str, default="json_data")
    parser.add_argument("--out", type=str, default="json_out")
    parser.add_argument("--normal_data", type=str, default="normal_data")
    parser.add_argument("--defect_data", type=str, default="defect_data")
    parser.add_argument("--mask_data", type=str, default="mask_data")
    parser.add_argument("--method", type=int, default=3,
                        help="表示采用的缺陷提取的方法为第几种：1~3,方法的含义参照相应的README.md")
    parser.add_argument("--defect_nums", type=int, default=1,help="最多融合的缺陷个数，<=2")
    parser.add_argument("--times", type=int, default=1000, help="每次执行后循环的次数")
    parser.add_argument("--fusion-only",type=bool,default=False)
    return parser.parse_args()
