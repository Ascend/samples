
import tensorflow as tf
from tensorflow.contrib.layers import fully_connected
import os
from sklearn.metrics import roc_auc_score
import numpy as np
from data_processing import record_train_test_split, record_to_dict, reduced_feature
from load_data import DataLoader
from evaluate import get_performance
import heapq
from arguments import rec_args
import time
import random
from tqdm import tqdm
from npu_bridge.estimator import npu_ops
from tensorflow.core.protobuf.rewriter_config_pb2 import RewriterConfig
import time
from tensorflow.python.tools import freeze_graph
from npu_bridge.npu_init import * 

config = tf.ConfigProto()
custom_op =  config.graph_options.rewrite_options.custom_optimizers.add()
custom_op.name =  "NpuOptimizer"
custom_op.parameter_map["use_off_line"].b = True #在昇腾AI处理器执行训练
# custom_op.parameter_map["profiling_mode"].b = True  # 打开profiling
# custom_op.parameter_map["profiling_options"].s = tf.compat.as_bytes('{"output":"/home/xidian/code/ARLMR-new/profiling","training_trace":"on","task_trace":"on","aicpu":"on","fp_point":"MobilenetV2/Conv/Conv2D","bp_point":"gradients/MobilenetV2/Conv/Conv2D_grad/Conv2DBackpropFilter"}')
config.graph_options.rewrite_options.remapping = RewriterConfig.OFF  #关闭remap开关

@tf.custom_gradient
def gather_npu(params, indices):
    def grad(dy):
        params_shape = tf.shape(params, out_type=tf.int64)
        params_shape = tf.cast(params_shape, tf.int32)
        grad_gather = tf.unsorted_segment_sum(dy, indices, params_shape[0])
        return grad_gather, None
    return tf.gather(params, indices), grad

class BaseModel(object):
    def __init__(self, args):
        # 加载交互记录
        # 超参
        self.args = args
        self.latent_vector_dim = args.dl_latent_vector_dim
        self.lr = args.dl_lr
        self.max_epoch = args.dl_max_epoch
        self.Ks = [1, 3, 10]
        self.batch_size = args.dl_batch_size
        self.reg = args.dl_reg
        self.record = record_to_dict(os.path.join(args.data_dir, args.record_csv))

        print('record loaded.')

        self.DataLoader = DataLoader(self.record, batch_size=self.batch_size)
        self.user_num = self.DataLoader.user_num
        self.item_num = self.DataLoader.item_num

        if args.item_feature == True:
            # item_features = reduced_feature(os.path.join(args.data_dir, args.item_feature_npy), keep_ratio=0.95)
            item_features = np.load(os.path.join(args.data_dir, args.item_feature_npy))
            self.item_content_dim = len(item_features[0])
            self.item_features = np.concatenate(([[0.0] * self.item_content_dim], item_features), axis=0).astype("float32")  # dim0 is reserved for None history users.
        elif args.item_feature == False :
            self.item_content_dim = 10
            self.item_features = np.array([[0.0 for _ in range(self.item_content_dim)]] * (self.item_num + 1)).astype("float32")
        else:
            exit()

        if args.user_feature == True:
            # user_features = reduced_feature(os.path.join(args.data_dir, args.user_feature_npy), keep_ratio=0.95)
            user_features = np.load(os.path.join(args.data_dir, args.user_feature_npy))
            # user_features = os.path.join(args.data_dir, args.user_feature_npy)
            self.user_content_dim = len(user_features[0])
            self.user_features = np.concatenate(([[0.0] * self.user_content_dim], user_features), axis=0).astype("float32")  # dim0 is reserved for None history users.
        elif args.user_feature == False:
            self.user_content_dim = 10
            self.user_features = np.array([[0.0 for _ in range(self.user_content_dim)]] * (self.user_num + 1)).astype("float32")
        else:
            exit()
        print('feature loaded.')

        self.create_graph()
        print('graph created.')
   
    def create_graph(self):
        w_init = tf.contrib.layers.xavier_initializer()
        l2 = tf.contrib.layers.l2_regularizer(self.reg)

        # 输入
        self.user_id = tf.placeholder(tf.int32, [64, 1])
        self.history_id = tf.placeholder(tf.int32, [64, 2000])
        self.history_len = tf.placeholder(tf.int32, [64, 1])
        self.positive_item_id = tf.placeholder(tf.int32, [64, 1])
        self.negative_item_id = tf.placeholder(tf.int32, [64, 1])
        
        self.user_feature = gather_npu(self.user_features, self.user_id)
        self.history_feature = gather_npu(self.item_features, self.history_id)
        self.pos_item_feature = gather_npu(self.item_features, self.positive_item_id)
        self.neg_item_feature = gather_npu(self.item_features, self.negative_item_id)

        # 隐特征及查找
        self.item_latent_vector = tf.Variable(tf.truncated_normal(shape=[self.item_num + 1, self.latent_vector_dim]), trainable=True)
        self.user_latent_vector = tf.Variable(tf.truncated_normal(shape=[self.user_num + 1, self.latent_vector_dim]), trainable=True)
        self.target_latent_vector = tf.Variable(tf.truncated_normal(shape=[self.item_num + 1, self.latent_vector_dim]), trainable=True)

        self.user_cf_vector = gather_npu(self.user_latent_vector, self.user_id)
        self.history_cf_vector = gather_npu(self.item_latent_vector, self.history_id)
        self.pos_item_cf_vector = gather_npu(self.target_latent_vector, self.positive_item_id)
        self.neg_item_cf_vector = gather_npu(self.target_latent_vector, self.negative_item_id)

        # 映射内容特征
        self.user_cb_vector = fully_connected(self.user_feature, self.latent_vector_dim, weights_initializer=w_init, weights_regularizer=l2)
        self.history_cb_vector = fully_connected(self.history_feature, self.latent_vector_dim, weights_initializer=w_init, weights_regularizer=l2)
        self.pos_item_cb_vector = fully_connected(self.pos_item_feature, self.latent_vector_dim, weights_initializer=w_init, weights_regularizer=l2, scope='content_map')
        self.neg_item_cb_vector = fully_connected(self.neg_item_feature, self.latent_vector_dim, weights_initializer=w_init, weights_regularizer=l2, reuse=True, scope='content_map')

        # 拼接content based特征和collaborative filtering特征
        self.user_vector = tf.squeeze(tf.concat([self.user_cf_vector, self.user_cb_vector], axis=2), axis=1)
        self.history_vector = tf.concat([self.history_cf_vector, self.history_cb_vector], axis=2)
        self.pos_item_vector = tf.squeeze(tf.concat([self.pos_item_cf_vector, self.pos_item_cb_vector], axis=2), axis=1)
        self.neg_item_vector = tf.squeeze(tf.concat([self.neg_item_cf_vector, self.neg_item_cb_vector], axis=2), axis=1)

        # 可能有填充情况
        history_len = tf.reduce_sum(self.history_len, 1)
        mask_mat = tf.expand_dims(tf.sequence_mask(history_len, maxlen=2000, dtype=tf.float32), -1)  # (b, n)
        self.real_history_vector = mask_mat * self.history_vector
        self.history_ave = tf.reduce_mean(self.real_history_vector, axis=1)

        # 生成策略
        self.user_exp = tf.concat([self.user_vector,
                                  tf.multiply(self.user_vector, self.history_ave),
                                  self.history_ave], axis=1)  # b, 3e

        first_relu = tf.layers.dense(self.user_exp, 4 * self.latent_vector_dim, tf.nn.relu,
                                     kernel_initializer=w_init, kernel_regularizer=l2, name='first_relu')
        second_relu = tf.layers.dense(first_relu, 4 * self.latent_vector_dim, tf.nn.relu,
                                      kernel_initializer=w_init, kernel_regularizer=l2, name='second_relu')
        self.user = tf.layers.dense(second_relu, 2 * self.latent_vector_dim,
                                    kernel_initializer=w_init, kernel_regularizer=l2, name='action')  # b, e
        # 输出
        self.pos_output = tf.diag_part(tf.matmul(self.user, tf.transpose(self.pos_item_vector)), name='output')
        self.neg_output = tf.diag_part(tf.matmul(self.user, tf.transpose(self.neg_item_vector)))

        self.bpr_loss = tf.negative(tf.reduce_mean(tf.log(tf.nn.sigmoid(self.pos_output - self.neg_output))))
        self.io_loss = self.reg * tf.nn.l2_loss(self.user_latent_vector) + self.reg * tf.nn.l2_loss(self.item_latent_vector)
        self.l2_loss = tf.add_n(tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES)) / self.batch_size
        self.loss = self.bpr_loss + self.l2_loss + self.io_loss
        self.opt = tf.train.AdamOptimizer(self.lr).minimize(self.loss)

    def predict_one_user(self, user, history, items):
        n = len(items)
        batch_size = 64
        index = 0
        ratings = []
        print ("当前时间戳为:", time.time())
        while index < n - batch_size:
            start = index
            end = index + batch_size
            real_batch_size = end - start
            batch_ratings = self.sess.run(self.pos_output, feed_dict={self.user_id: [[user]] * real_batch_size,
                                                                      self.history_id: [history] * real_batch_size,
                                                                      self.history_len: [[len(history)]] * real_batch_size,
                                                                      self.positive_item_id: np.reshape(items[start:end], [-1, 1])})
            ratings += list(batch_ratings.flatten())
            index += batch_size
        print ("当前时间戳为:", time.time())
        return ratings

    def get_user_seq(self, user):
        history = self.DataLoader.get_history(user)
        history_1 = list(map(lambda x: x + [0] * (2000 - len(x)), [history]))
        # print(history_1[0])
        history = history_1[0]
        all_items = set(range(1, self.item_num+1))
        items = list(all_items - set(history))
        test_pos = self.DataLoader.get_test_items(user)
        ratings = self.predict_one_user(user, history, items)
        # 得到topK的排序, 并标记好命中位置
        k_max = max(self.Ks)
        K_max_score_item = heapq.nlargest(k_max, zip(ratings, items))
        ranked_K_max_item = [v[1] for v in sorted(K_max_score_item)]
        r = [1 if i in test_pos else 0 for i in ranked_K_max_item]
        return r

    def eval(self):
        precision = np.array([0.0 for i in range(len(self.Ks))])
        recall = np.array([0.0 for i in range(len(self.Ks))])
        hr = np.array([0.0 for i in range(len(self.Ks))])
        for user in self.record:
            print('\revaluating user {} / {}'.format(user, self.user_num), end='', flush=True)
            r = self.get_user_seq(user)
            performance = get_performance(self.DataLoader.get_test_items(user), r, self.Ks)
            precision += performance['precision']
            recall += performance['recall']
            hr += performance['hit_ratio']
        return precision / self.user_num, recall / self.user_num, hr / self.user_num

    def approximate_eval(self):
        auc = 0
        eval_num = len(self.record)
        user_num = 0
        for user in self.record:
            user_num += 1
            print('\revaluating user {} / {}'.format(user_num, eval_num), end='', flush=True)
            test_pos = self.DataLoader.get_test_items(user)
            n = len(test_pos)
            test_neg = [self.DataLoader.neg_sample(user) for i in range(n)]
            history = self.DataLoader.get_history(user)
            eval_items = test_pos + test_neg
            eval_labels = [[1]] * n + [[0]] * n
            eval_ratings = self.predict_one_user(user, history, eval_items)
            try:
                auc += roc_auc_score(eval_labels, eval_ratings)
            except:
                print("gradient explode, train stop.")
                break
        return auc / eval_num

    def baseline_auc(self):
        auc = 0
        eval_num = len(self.record)
        user_num = 0
        print('\revaluating user ...')
        for user in tqdm(self.record):
            user_num += 1
            # print('\revaluating user {} / {}'.format(user_num, eval_num), end='', flush=True)
            test_pos = self.DataLoader.get_test_items(user)
            n = len(test_pos)
            eval_labels = [[1]] * n + [[0]] * n
            eval_ratings = [[1]] * n + [[0]] * n
            random.shuffle(eval_ratings)
            auc += roc_auc_score(eval_labels, eval_ratings)
        return auc / eval_num

    def train(self):
        print('start train...')
        performance_record = []
        with tf.Session(config=config) as self.sess:
            self.sess.run(tf.global_variables_initializer())
            # tf.io.write_graph(self.sess.graph_def, './', 'graph.pbtxt')
            saver = tf.train.Saver(max_to_keep=1)
            max_auc = 0
            print('baseline auc:', self.baseline_auc())
            # print(self.eval())
            print('start training.')
            for epoch in range(self.max_epoch):
                # 打乱训练数据顺序
                self.DataLoader.shuffle()
                batch = self.DataLoader.generate_train_batch()
                # print(time.asctime( time.localtime(time.time())))
                # 训练
                try:
                    while 1:
                        user_batch, history_batch, len_batch, pos_batch, neg_batch = next(batch)
                        self.sess.run(self.opt, feed_dict={self.user_id: np.reshape(user_batch, [-1, 1]),
                                                           self.history_id: history_batch,
                                                           self.history_len: np.reshape(len_batch, [-1, 1]),
                                                           self.positive_item_id: np.reshape(pos_batch, [-1, 1]),
                                                           self.negative_item_id: np.reshape(neg_batch, [-1, 1])})
                        # print ("当前时间戳为:", time.time())
                        # print('#################')
                except StopIteration:
                    print('epoch: {} finish!'.format(epoch))
                    saver.save(self.sess, './ckpt/MM.ckpt')
            model_file = tf.train.latest_checkpoint('./ckpt/')
            saver.restore(self.sess, model_file)
#             print('999')
            tf.train.write_graph(self.sess.graph.as_graph_def(), './pb_model', 'model.pbtxt', as_text=True) 
            ckpt_path = "/home/xidian/code/ARLMR-new/ckpt/MM.ckpt"
            freeze_graph.freeze_graph(
		        input_graph='./pb_model/model.pbtxt',   # 传入write_graph生成的模型文件
		        input_saver='',
		        input_binary=False, 
		        input_checkpoint=ckpt_path,  # 传入训练生成的checkpoint文件
		        output_node_names='output',  # 与定义的推理网络输出节点保持一致
		        restore_op_name='save/restore_all',
		        filename_tensor_name='save/Const:0',
		        output_graph='./pb_model/MM-np32.pb',   # 改为需要生成的推理网络的名称
		        clear_devices=False,
		        initializer_nodes='')
#             final_auc = self.eval()
            # # print('final performance: auc:{}'.format(final_auc))
            # print(final_auc)
            self.feature_gen()
            print('feature_gen finish!')

            del self.item_features
            del self.user_features
            del self.DataLoader
        # return precision, recall, hr, ndcg

    def feature_gen(self):
        if not os.path.exists('./multi_source_feature'):
            os.makedirs('./multi_source_feature')
        multisource_user_feature = np.array([[0.0 for _ in range(2 * self.latent_vector_dim)]] * (self.user_num + 1))
        multisource_history_feature = np.array([[0.0 for _ in range(2 * self.latent_vector_dim)]] * (self.item_num + 1))
        multisouce_target_feature = np.array([[0.0 for _ in range(2 * self.latent_vector_dim)]] * (self.item_num + 1))
        for user in range(self.user_num + 1):
            print('\rgenerate feature: user {} / {}'.format(user, self.user_num), end='', flush=True)
#             print(self.sess.run(self.user_vector, feed_dict={self.user_id: [[user]] * 64}).shape)
            multisource_user_feature[user] = np.ravel(self.sess.run(self.user_vector, feed_dict={self.user_id: [[user]] * 64})[0])
        np.save('./multi_source_feature/user_feature.npy', multisource_user_feature)
        for item in range(self.item_num + 1):
            print('\rgenerate feature: item {} / {}'.format(item, self.item_num), end='', flush=True)
#             print(self.sess.run(self.history_vector, feed_dict={self.history_id: [[item]*2000] * 64}).shape)
            multisource_history_feature[item] = np.ravel(self.sess.run(self.history_vector, feed_dict={self.history_id: [[item]*2000] * 64})[0][0])
            multisouce_target_feature[item] = np.ravel(self.sess.run(self.pos_item_vector, feed_dict={self.positive_item_id: [[item]] * 64})[0])
        np.save('./multi_source_feature/history_feature.npy', multisource_history_feature)
        np.save('./multi_source_feature/target_feature.npy', multisouce_target_feature)

if __name__ == '__main__':
    args = rec_args()
    m = BaseModel(args)
    m.train()

