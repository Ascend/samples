"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-11 10:12:13
MODIFIED: 2020-12-11 14:04:45
"""
import sys
import math
import operator
import functools 
import hashlib

def file_compare(testfile):
    """
    file read md5
    """
    md5=hashlib.md5()
    with open(testfile, "r") as f1:
        while True:
            b=f1.read(8096)
            if not b:
                break
            md5.update(b.encode('utf-8'))
    f1.close()
    return md5.hexdigest()  


def file_contrast(fileA, fileB):
    """
    Verify that the pictures are the same
    """
    file1_md5 = file_compare(fileA)
    file2_md5 = file_compare(fileB)
    if file1_md5 == file2_md5:
        print("file_compare1: the two file is sample")
        return True 
    else:
        print("file_compare1: the two file is different")
        return False 

if __name__ == '__main__':
    txt1 = sys.argv[1]
    txt2 = sys.argv[2]
    result = file_contrast(txt1, txt2)
    if (result != True):
        sys.exit(1)
    else:
        sys.exit(0)
