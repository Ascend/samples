# -*- coding: utf8 -*-
import os
import random
import tensorflow as tf

from tensorflow.keras.optimizers import Adam
import numpy as np
from keras.models import Sequential
from keras.wrappers.scikit_learn import KerasClassifier
from keras.utils import np_utils
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from keras.layers import Dense,Flatten,Conv2D,MaxPool2D
from keras.models import load_model

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--feature_dir", default="")
parser.add_argument("--output_path", default="")
args = parser.parse_args()

result = [[] for i in range(29)]

def seed_tensorflow(seed):
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
    tf.random.set_seed(seed)

def readFile(path):
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
    b = [227,1482,459,176,328,1492,1130,117,1472,621,416,276,365,983,1244,879,795,507,1514,612]
    for x in range(0,29):
        result = [float(matrix[x][c]) for c in b]
        a.append(result)
    arr=np.array(a)
    f.close()
    return arr

def readFile_byindex(index,path):
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
    b = [227,1482,459,176,328,1492,1130,117,1472,621,416,276,365,983,1244,879,795,507,1514,612]
    result = [float(matrix[index-1][c]) for c in b]
    f.close()
    return result

seed = 1
seed_tensorflow(seed)
for m in range(1,30):
    data = []
    Y_train = []
    for i in range(1,39):
        train_data = readFile_byindex(m,args.feature_dir + '/train/audio_feature0{}.txt'.format(i))
        data.append(train_data)
        if i < 17:
            Y_train.append(1)
        elif i>=17:
            Y_train.append(0)
    X_train=np.array(data)
    for i in range(0, 38):
        for j in range(0, 20):
            if float(X_train[i][j]) != 0.0:
                X_train[i][j] = float(format(X_train[i][j]+0.000000000000001, '.3g'))
    Y_train=np.array(Y_train)
    X_train=np.expand_dims(X_train[:, 0:20], axis=2)
    Y_train=Y_train.reshape((38,))

    encoder = LabelEncoder()
    Y_train_encoded = encoder.fit_transform(Y_train)
    Y_train = np_utils.to_categorical(Y_train_encoded)

    X_train, X_test, Y_train, Y_test = train_test_split(X_train, Y_train, test_size=0.25, random_state=5)

    X_train = np.expand_dims(X_train,axis=1).astype(np.float32)
    X_test = np.expand_dims(X_test, axis=1).astype(np.float32)
    test_label_1 = []
    test_label_2 = []
    for n in range(len(Y_test)):
        test_label_1.append(int(Y_test[n][1]))

    def baseline_model():
        model = Sequential()
        model.add(Conv2D(16, (3, 1), input_shape=(1, 20, 1), strides=(1, 1), padding='SAME', activation='relu'))
        model.add(Conv2D(32, (3, 1), padding='SAME', activation='tanh'))
        model.add(MaxPool2D((1, 2)))
        model.add(Conv2D(64, (3, 1), padding='SAME', activation='tanh'))
        model.add(Conv2D(64, (3, 1), padding='SAME', activation='tanh'))
        model.add(MaxPool2D((1, 2)))
        model.add(Flatten())
        model.add(Dense(2, activation='softmax'))
        model.compile(loss='categorical_crossentropy',optimizer=Adam(learning_rate=0.001), metrics=['accuracy'])
        return model

    estimator = KerasClassifier(build_fn=baseline_model, epochs=300, batch_size=1, verbose=1)
    estimator.fit(X_train, Y_train)

    estimator.model.save(args.output_path + "/model_{}.h5".format(m))

    loaded_model = load_model(args.output_path + "/model_{}.h5".format(m))
    loaded_model.compile(loss='categorical_crossentropy', optimizer=Adam(learning_rate=0.001), metrics=['accuracy'])
    scores = loaded_model.evaluate(X_test, Y_test, verbose=0)
    print('%s: %.2f%%' % (loaded_model.metrics_names[1], scores[1] * 100))
    predicted_label = loaded_model.predict(X_test)
    for o in range(len(predicted_label)):
        test_label_2.append(np.argmax(predicted_label[o],axis=0))
    #计算灵敏度
    q1 = 0 #真阳性人数
    q2 = 0 #假阴性人数
    result_1 = 0
    for i in range(len(test_label_1)):
        if test_label_1[i] == test_label_2[i] == 1:
            q1 += 1
        if test_label_1[i] == 1 and test_label_2[i] == 0:
            q2 += 1
    result_1 = q1 / (q1 + q2)
    print('模型灵敏度：' + str(result_1))

    #计算特异性
    p1 = 0 #真阴性人数
    p2 = 0 #假阳性人数
    result_2 = 0
    for j in range(len(test_label_1)):
        if test_label_1[j] == test_label_2[j] == 0:
            p1 += 1
        if test_label_1[j] == 0 and test_label_2[j] == 1:
            p2 += 1
    result_2 = p1 / (p1 + p2)
    print('模型特异性：' + str(result_2))

    result[m-1].append(result_1)
    result[m-1].append(result_2)

result_specificity = np.array(result)
result_specificity = np.lexsort(-result_specificity.T)
for i in range(len(result_specificity)):
    result_specificity[i] += 1

result_sensitivity = np.array(result)
result_sensitivity = np.lexsort(-result_sensitivity[:,::-1].T)
for i in range(len(result_sensitivity)):
    result_sensitivity[i] += 1

print(result_specificity)
print(result_sensitivity)

all_result = []

index_Threshold = 2

for idx in range(1,14):
    data_test = readFile(args.feature_dir + '/test/audio_feature0{}.txt'.format(idx))
    for i in range(0, 29):
        for j in range(0, 20):
            if float(data_test[i][j]) != 0.0:
                data_test[i][j] = float(format(data_test[i][j]+0.000000000000001, '.3g'))

    health = 0
    depression = 0
    i = 0 #灵敏度
    j = 0 #特异性
    model_indexs = []
    model_indexs.append(result_sensitivity[0])
    model_indexs.append(result_sensitivity[1])
    for i in range(29):
        if result_specificity[i] not in model_indexs:
            model_indexs.append(result_specificity[i])
            break
    #print(model_indexs)

    for model in model_indexs:
        loaded_model =load_model(args.output_path + "/model_{}.h5".format(model))
        loaded_model.compile(loss='categorical_crossentropy', optimizer=Adam(learning_rate=0.001), metrics=['accuracy'])
        if int(np.argmax(loaded_model.predict(np.array(data_test[model-1]).reshape((1,1,20,1))),axis=1)) == 0:
            health += 1
        else :
            depression += 1
    if health >= index_Threshold:
        all_result.append(0)
    else :
        all_result.append(1)

print(all_result)

res = 0

for i in range(0,6):
    if all_result[i] == 1:
        res +=1
for i in range(6,13):
    if all_result[i] == 0:
        res +=1
print('acc：' + str(res/13))