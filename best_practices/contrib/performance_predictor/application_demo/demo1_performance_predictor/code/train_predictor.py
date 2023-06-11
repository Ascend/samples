 #coding=utf-8
import tensorflow as tf
import json
import re
import numpy as np
import random
from sklearn.metrics import r2_score
from scipy.stats import kendalltau
from sklearn import ensemble
import joblib


#邻接矩阵填充
def padding_zero_in_matrix(adjecent_matrix, module_operations):
    len_operations = len(module_operations)
    if len_operations != 7:
        # if the operations is less than 7
        for j in range(len_operations, 7):
            module_operations.insert(j - 1, 3)
        # print(important_metrics[i]['fixed_metrics']['module_operations'])

        padding_matrix = np.insert(adjecent_matrix, len_operations - 1,
                                       np.zeros([7 - len_operations, len_operations]), axis=0)
        padding_matrix = np.insert(padding_matrix, [len_operations - 1], np.zeros([7, 7 - len_operations]), axis=1)
    else:
        padding_matrix = np.array(adjecent_matrix)
    return padding_matrix, module_operations

#编码方法
def encode(adjecent_matrix, module_operations):
    #填充架构为相同大小的邻接矩阵和相同长度的节点标签列表
    padding_matrix, module_operations = padding_zero_in_matrix(adjecent_matrix, module_operations)
    #去除节点标签列表的in节点和out节点
    module_operations.remove(-1)
    module_operations.remove(-2)
    #将节点标签列表变成one-hot矩阵
    operations = []
    for i in range(0,len(module_operations)):
        ops = [ 0 for i in range(0,5)]
        if i < 3:
            ops[i] = 1
            operations.append(ops)
        else:
            operations.append(ops)
    operations = np.array(operations)
    #将邻接矩阵和节点标签列表化为一维并连接
    adj = list(np.ndarray.flatten(padding_matrix))
    operation = list(np.ndarray.flatten(operations))
    code = np.append(adj,operation)
    return code


#数据加载
def dataset_load(data_dir="data/data.json", graph_dir="data/generate_graphs.json", train_num=2500, test_num=500):
    #加载数据集中的key及对应的评估信息。
    model_id_regex="^"
    regex = re.compile(model_id_regex)
    with tf.gfile.Open(data_dir) as f:
        exist_models = json.load(f)
    exist_keys = [key for key in exist_models.keys() if regex.match(key)]
    random.shuffle(exist_keys)

    #分配训练集的key和测试集的key
    train_keys = exist_keys[0:train_num]
    test_keys = exist_keys[train_num:train_num+test_num]

    #获得key对应的架构的邻接矩阵和节点标签
    with tf.gfile.Open(graph_dir) as f:
        all_models = json.load(f)

    #分配训练集的邻接矩阵和节点标签以及标签（即性能）
    train_matrix = []
    train_ops = []
    train_label = []
    for i in range(len(train_keys)):
        key = train_keys[i]
        acc = exist_models[key]['evaluation_results'][2]['validation_accuracy']
        matrix = all_models[key][0]
        ops = all_models[key][1]
        train_matrix.append(matrix)
        train_ops.append(ops)
        train_label.append(acc)

    #分配测试集的邻接矩阵和节点标签以及标签（即性能）
    test_matrix = []
    test_ops = []
    test_label = []
    for i in range(len(test_keys)):
        key = test_keys[i]
        acc = exist_models[key]['evaluation_results'][2]['test_accuracy']
        matrix = all_models[key][0]
        ops = all_models[key][1]
        test_matrix.append(matrix)
        test_ops.append(ops)
        test_label.append(acc)
    return train_matrix, train_ops, train_label, test_matrix, test_ops, test_label

#获取模型
def get_model():
    model_random_forest_regressor = ensemble.RandomForestRegressor(n_estimators=230,max_features='auto')
    return model_random_forest_regressor

#数据编码
def encode_data(train_matrix, train_ops, train_label, test_matrix, test_ops, test_label):
    x_train = []
    for i in range(0,len(train_matrix)):
        x_train.append(encode(train_matrix[i],train_ops[i]))
    x_test = []
    for i in range(0,len(test_matrix)):
        x_test.append(encode(test_matrix[i],test_ops[i]))
    return x_train, train_label, x_test, test_label
    


#训练模型
def try_model(x_train, y_train, x_test, y_test, model):
    model.fit(x_train, y_train)
    result = model.predict(x_test)
    result = list(result)
    score = r2_score(y_test, result)
    result_arg = np.argsort(result)
    y_test_arg = np.argsort(y_test)
    result_rank = np.zeros(len(y_test_arg))
    y_test_rank = np.zeros(len(y_test_arg))
    for i in range(len(y_test_arg)):
        result_rank[result_arg[i]] = i
        y_test_rank[y_test_arg[i]] = i
    KTau, _ = kendalltau(result_rank, y_test_rank)
    joblib.dump(model, "output/RF_model.m") #save model
    print('KTau: {:}, R2score: {:}'.format(KTau, score))



if __name__ == '__main__':
    data_dir = "data/data.json"
    graph_dir = "data/generated_graphs.json"
    train_matrix, train_ops, train_label, test_matrix, test_ops, test_label = dataset_load(data_dir, graph_dir, 2000, 500)
    x_train, y_train, x_test, y_test = encode_data(train_matrix, train_ops, train_label, test_matrix, test_ops, test_label)
    model = get_model()
    try_model(x_train, y_train, x_test, y_test, model)
    
