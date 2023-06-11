import joblib
from train_predictor import encode


def get_model(dir="output/RF_model.m"):
    model = joblib.load(dir)
    return model 

def predict_data(model, adj, operation):
    encoding = encode(adj, operation)
    result = model.predict([encoding])
    print("输入架构信息:")
    print("邻接矩阵:"+str(adj))
    print("操作列表:"+str(operation))
    print("输入架构的验证准确度预测为:"+str(result[0]))
    return result


if __name__ == '__main__':
    model = get_model()
    adj = [[0, 1, 1, 1, 0, 0], [0, 0, 0, 0, 1, 1], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0]]
    operation = [-1, 1, 0, 0, 1, -2]
    performance = predict_data(model, adj, operation)