from generate_pkl import generate_pkl
from query_101 import nas_bench_101_api

# generate the pkl file for nas bench 101 data
generate_pkl(graph_path='data/generated_graphs.json', data_path='data/data.json', target_path='data_pack_101.pkl')

# demo of querying one architecture
arch = [[[0, 1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0, 0], [0, 0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0]], [-1, 1, 2, 0, 1, 0, -2]]

arch_information = nas_bench_101_api(data_path='data_pack_101.pkl', arch=arch)

print('Architecture: ', arch)
print('Trainable_params: ', arch_information['trainable_params'])
print('Total training time: ', arch_information['total_time'])
print('Evaluation results: ')
print(arch_information['evaluation_results'][0])
print(arch_information['evaluation_results'][1])
print(arch_information['evaluation_results'][2])