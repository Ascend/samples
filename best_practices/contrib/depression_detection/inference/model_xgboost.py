# plot feature importance manually
# _*_ coding:utf8 _*_
import numpy as np
from xgboost import XGBClassifier
from xgboost import plot_importance
from matplotlib import pyplot

def readFile(idx,path):
    f = open(path)
    lines = f.readlines()
    del lines[0:1589]
    first_ele = True
    for data in lines:
        data = data.strip('\n')
        nums = data.split(',')
        if first_ele:
            matrix = np.array(nums)
            first_ele = False
        else:
            matrix = np.c_[matrix,nums]
    matrix = matrix.transpose()
    a = []
    #if idx != 4:
    for x in range(0,29):
        result = [float(matrix[x][c]) for c in range(1,1583)]
        if idx < 17:
            result.append(1)
        else :
            result.append(0)
        a.append(result)
    # else :
    #     for x in range(0,24):
    #         result = [float(matrix[x][c]) for c in range(1,1583)]
    #         if idx < 24:
    #             result.append(1)
    #         else :
    #             result.append(0)
    #         a.append(result)
    arr=np.array(a)
    f.close()
    return arr

data1 = []
data3 = []

for i in range(1,39):
    data = readFile(i,r'E:\pythonProject\Audioprocessing\audio_data\audio_data\data_featrue_final\train\audio_feature0{}.txt'.format(i))#E:\pythonProject\Audioprocessing\audio_data\audio_data\data_featrue_final\train
    print(data.shape)
    data1.append(data)

data_1 = np.array(data1)
print(data_1.shape)
data_train = data_1.reshape(38*29,1583)

# data2 = readFile(4,r'C:\Users\11496\Desktop\audio_feature\audio_feature04.txt')
# data_2 = np.array(data2)
#
# data_train = np.vstack((data_1,data_2))
#
# test_index = [1,3,5,6,19,21,26,30,31,34,39,42,46]
# for i in range(5,53):
#     if i not in test_index:
#         data = readFile(i,r'C:\Users\11496\Desktop\audio_feature\audio_feature0{}.txt'.format(i))
#         data3.append(data)
# data_3 = np.array(data3)
# data_3 = data_3.reshape(37*29,1583)
#
# data_train = np.vstack((data_train,data_3))

for i in range(0,1102):
    for j in range(0,1583):
        if float(data_train[i][j])!= 0.0:
            data_train[i][j] = float(format(data_train[i][j]+0.000000000000001,'.3g'))

#
#
# 划分样本数据和标签
x,y=np.split(data_train,indices_or_sections=(1582,),axis=1)
# # fit model no training data
model = XGBClassifier()
model.fit(x, y)
# feature importance
print(model.feature_importances_)
# plot
plot_importance(model,importance_type="gain",max_num_features=20)
pyplot.savefig('feature_gain_test.jpg')
pyplot.show()