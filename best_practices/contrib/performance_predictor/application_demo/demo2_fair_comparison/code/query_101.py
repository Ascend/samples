import pickle

def nas_bench_101_api(data_path, arch):
    data_pack_101 = open(data_path, 'rb')
    data_pack_101_dict = pickle.load(data_pack_101)

    graph_id = None
    for key, value in data_pack_101_dict['generated_graphs_dict'].items():
        if value == arch:
            graph_id = key

    if graph_id not in data_pack_101_dict['data_dict']:
        print('This architecture is not included! Please refer to others...')
    else:
        arch_information = data_pack_101_dict['data_dict'][graph_id]
        return arch_information