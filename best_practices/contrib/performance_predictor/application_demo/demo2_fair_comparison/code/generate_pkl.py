import json
import pickle

def generate_pkl(graph_path, data_path, target_path):
    generated_graphs = open(graph_path, 'r')
    data = open(data_path, 'r')

    generated_graphs_dict = json.load(generated_graphs)
    data_dict = json.load(data)

    data_pack_101 = {'generated_graphs_dict': generated_graphs_dict, 'data_dict': data_dict}

    data_pack_101_pth = open(target_path, 'wb')
    pickle.dump(data_pack_101, data_pack_101_pth)