# -*- coding: UTF-8 -*-
import sys
sys.path.append("../../../../python/common/")
sys.path.append("../../../../python/common/acllite")
sys.path.append("../")
import numpy as np
import os
#from subprocess import call
#import sys

#path = os.path.dirname(os.path.abspath(__file__))
#sys.path.append(os.path.join(path, "samples/python/common/acllite"))

import argparse
import acl
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
from acllite_image import AclLiteImage

parser = argparse.ArgumentParser()
parser.add_argument("--feature_dir", default="")
parser.add_argument("--model_path", default="")
args = parser.parse_args()

def readFile(path):
    f = open(path)
    lines = f.readlines()
    del lines[0:1589]
    first_ele = True
    for data in lines:
        data = data.strip('\n')
        nums = data.split(',')
        ## 添加到 matrix 中。
        if first_ele:
            ### 加入到 matrix 中 。
            matrix = np.array(nums)
            first_ele = False
        else:
            matrix = np.c_[matrix,nums]
    matrix = matrix.transpose()
    a = []
    b = [227,1482,459,176,328,1492,1130,117,1472,621,416,276,365,983,1244,879,795,507,1514,612]
    for x in range(0,3):
        result = [float(matrix[x][c]) for c in b]
        a.append(result)
    arr=np.array(a)
    f.close()
    return arr

def main():
    res = []
    acl_resource = AclLiteResource()
    acl_resource.init()
    
    idx = [1,2]
    for i in idx:
        data_test = readFile(args.feature_dir + '/audio_feature0{}.txt'.format(i))
        x,y = np.split(data_test,indices_or_sections=(20,),axis=1)
        x = x[:,0:20]
        for i in range(0, 3):
            for j in range(0, 20):
                if float(x[i][j]) != 0.0:
                    x[i][j] = float(format(x[i][j] + 0.000000000000001, '.3g'))
        health = 0
        depression = 0
        for idx in range(1,4):
            data_arr = np.array(x[idx-1], dtype = np.float32)
            input_blob = np.expand_dims(data_arr,axis=0).astype(np.float32)
            input_blob = input_blob.reshape(1,1,20,1)
            model = AclLiteModel(args.model_path + '/tf_model_{}.om'.format(idx))
            output = model.execute(input_blob)
            out = np.array(output[0][0])
            if int(np.argmax(out,axis=0)) == 0:
                health += 1
            else :
                depression += 1
        if health >= 2:
            res.append(0)
        else :
            res.append(1)   
    
    print(res)
    result = 0
    if res[0] == 1:
        result +=1
    if res[1] == 0:
        result += 1
    print('acc:' + str(result/2))
    
if __name__== "__main__" :
    main()