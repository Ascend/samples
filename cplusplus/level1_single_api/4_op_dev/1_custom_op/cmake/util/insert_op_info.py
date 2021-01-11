# -*- coding: utf-8 -*-
"""
Created on Feb  28 20:56:45 2020
Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
"""
import json
import os
import sys

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(sys.argv)
        print('argv error, inert_op_info.py your_op_file lib_op_file')
        exit(2)

    with open(sys.argv[1], 'r') as load_f:
        insert_op = json.load(load_f)

    all_op = {}
    if os.path.exists(sys.argv[2]):
        if os.path.getsize(sys.argv[2]) != 0:
            with open(sys.argv[2], 'r') as load_f:
                all_op = json.load(load_f)

    for k in insert_op.keys():
        if k in all_op.keys():
            print('replace op:[', k, '] success')
        else:
            print('insert op:[', k, '] success')
        all_op[k] = insert_op[k]

    with open(sys.argv[2], 'w') as f:
        f.write(json.dumps(all_op, indent=4))
