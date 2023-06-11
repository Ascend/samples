# NAS-Bench-101数据包生成及使用示例

## 1.数据准备

data/generated_graphs.json: 包含NAS-bench-101搜索空间中的全部423624条架构信息；使用`python generate_graph.py`命令生成

data/data.json: 包含查询的架构的详细信息，包括每个架构的训练相关信息及性能相关信息。

## 2.数据打包生成pkl文件

根据上述两个json文件的文件路径，以及生成的pkl数据文件要输出到的路径，使用generate_pkl.py中的方法进行数据打包，如下所示：

    # generate the pkl file for nas bench 101 data
    generate_pkl(graph_path, data_path, target_path)

## 3.使用打包后的pkl数据文件进行架构信息查询

根据打包后的pkl数据文件，输入要查询的架构信息，使用query_101.py中的方法进行架构信息的查询，如下所示：

    # demo of querying one architecture
    arch = [[[0, 1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0, 0], [0, 0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0]], [-1, 1, 2, 0, 1, 0, -2]]
    
    arch_information = nas_bench_101_api(data_path, arch=arch)

## 4.使用方法Demo

完整使用方法演示Demo在demo.py文件中呈现。

## 5.架构查询Demo运行结果

    Architecture:  [[[0, 1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0, 0], [0, 0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0]], [-1, 1, 2, 0, 1, 0, -2]]
    Trainable_params:  21547914
    Total training time:  2770.563542842865
    Evaluation results: 
    {'epochs': 0, 'training_steps': 0, 'train_accuracy': 0.09845753014087677, 'validation_accuracy': 0.1028645858168602, 'test_accuracy': 0.10056089609861374, 'predict_time': 32.889620780944824}
    {'epochs': 54, 'training_steps': 8437, 'train_accuracy': 0.35596954822540283, 'validation_accuracy': 0.3452523946762085, 'test_accuracy': 0.34515222907066345, 'predict_time': 34.821516036987305}
    {'epochs': 108, 'training_steps': 16875, 'train_accuracy': 0.9992988705635071, 'validation_accuracy': 0.8845152258872986, 'test_accuracy': 0.8683894276618958, 'predict_time': 34.703524351119995}

Architecture表示当前查询的架构编码。Trainable_params表示可训练参数数量，Total training time表示训练总耗时(s)。Evaluation results: 中存储第0，54，108个迭代后模型的性能指标，以迭代0为例，epoch表示当前迭代次数，training_steps表示当前训练的步数，train_accuracy表示当前迭代下模型在训练集上的准确率，validation_accuracy表示当前迭代下模型在验证集上的准确率，test_accuracy表示当前迭代下模型在测试集上的准确率，predict_time表示模型在测试数据集上的验证时间。