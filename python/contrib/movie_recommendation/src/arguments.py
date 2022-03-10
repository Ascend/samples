import argparse

def rec_args():
    parser = argparse.ArgumentParser(description="Recommendation base on Reinforcement Learning.")
    parser.add_argument('--rl_batch_size', default=64, type=int,
                        help='batch size in reinforcement learning')
    parser.add_argument('--rl_gamma', default=0.9, type=float,
                        help='gamma in reinforcement learning')
    parser.add_argument('--rl_actor_lr', default=10e-8, type=float,
                        help='actor learning rate in reinforcement learning')
    parser.add_argument('--rl_critic_lr', default=10e-8, type=float,
                        help='critic learning rate in reinforcement learning')
    parser.add_argument('--rl_e_greedy', default=0.001, type=float,
                        help='e greedy parameter in reinforcement learning')
    parser.add_argument('--rl_memory_capacity', default=100000, type=int,
                        help='size of pos memory and neg memory')
    parser.add_argument('--rl_neg_pos_ratio', default=2, type=int,
                        help='ratio between neg experience and pos experience')
    parser.add_argument('--dl_latent_vector_dim', default=128, type=int,
                        help='latent vector size')
    parser.add_argument('--dl_lr', default=0.001, type=float,
                        help='learning rate of deep learning')
    parser.add_argument('--dl_max_epoch', default=10, type=int,
                        help='max epoch number of deep learning')
    parser.add_argument('--dl_batch_size', default=64, type=int,
                        help='batch_size in deep learning')
    parser.add_argument('--dl_reg', default=2e-5, type=float,
                        help='regularization')
    parser.add_argument('--data_dir', default='./dataset/ml-1m/',
                        help='directory of user information, item information, record')
    parser.add_argument('--record_csv', default='user_item_history_1.csv',
                        help='record csv file')
    parser.add_argument('--item_feature_npy', default='item_feature.npy',
                        help='item feature npy file')
    parser.add_argument('--user_feature_npy', default='user_feature.npy',
                        help='user feature npy file')
    parser.add_argument('--user_info_csv', default='user_match.csv',
                        help='user information csv')
    parser.add_argument('--item_info_csv', default='item_match.csv',
                        help='item information csv')
    parser.add_argument('--rec_num', default=1, type=int,
                        help='recommend item numbers each user')
    parser.add_argument('--sim_recall', default=10, type=int,
                        help='use n most similar items as candidate set')

    parser.add_argument('--item_feature', default=True, type=bool,
                        help='True|False')
    parser.add_argument('--user_feature', default=True, type=bool,
                        help='True|False')
    # parser.add_argument('--discnt_data_dir', default='./output/discount/',
    #                     help='directory of user information, item information, record')
    # parser.add_argument('--discnt_record_csv', default='discount_user_items_history_map.csv',
    #                     help='record csv file')
    # parser.add_argument('--user_feature_npy', default='cmcc_all_discount_combined.npy',
    #                     help='item feature npy file')
    # parser.add_argument('--discnt_user_info_csv', default='discnt_matched_user_data.csv',
    #                     help='user information csv')
    # parser.add_argument('--discnt_item_info_csv', default='discnt_matched_item_data.csv',
    #                     help='item information csv')
    return parser.parse_args()