"""
Created on Oct 9, 2019.
Training Actor Critic network.

@author: Monday Ma.
"""

import tensorflow as tf
import numpy as np
import os
from Environment import Environment
import random
from data_processing import read_users_to_set, res_dic_to_csv
from arguments import rec_args
from tensorflow.contrib.layers import fully_connected
import gc
from tqdm import tqdm
from npu_bridge.estimator import npu_ops
from tensorflow.core.protobuf.rewriter_config_pb2 import RewriterConfig

config = tf.ConfigProto()
custom_op =  config.graph_options.rewrite_options.custom_optimizers.add()
custom_op.name =  "NpuOptimizer"
custom_op.parameter_map["use_off_line"].b = True #在昇腾AI处理器执行训练
config.graph_options.rewrite_options.remapping = RewriterConfig.OFF  #关闭remap开关

args = rec_args()

BATCH_SIZE = args.rl_batch_size
GAMMA = args.rl_gamma    # account rate
LR_A = args.rl_actor_lr   # learning rate for actor
LR_C = args.rl_critic_lr   # learning rate for critic
E_GREEDY = args.rl_e_greedy
memory_capacity = args.rl_memory_capacity
neg_pos_ratio = args.rl_neg_pos_ratio

env = Environment(args)
N_S = env.state_len
N_A = env.action_len

SESS = tf.Session(config=config)
OPT_A = tf.train.AdamOptimizer(LR_A)
OPT_C = tf.train.AdamOptimizer(LR_C)

if not os.path.exists('./feedback'):
    os.makedirs('./feedback')
if not os.path.exists('./result'):
    os.makedirs('./result')

user_info_path = os.path.join(args.data_dir, args.user_info_csv)
item_info_path = os.path.join(args.data_dir, args.item_info_csv)

class AC_net(object):
    """
    Build Actor Critic network and training.

    Attributes:
        s: State.
        target_q: Target q_value, for updating Critic network.
        action: Output of Actor.
        q_value: Output of Critic.
        a_params: Actor network parameters.
        c_params: Critic network parameters.
        update_c: Option for updating critic network.
        update_a: Option for updating actor network.
    """
    def __init__(self, scope):
        """
        Build Actor Critic network, define loss\gradient\optimizer

        Args:
            scope: Identifier of actor critic network
        """
        with tf.variable_scope(scope):
            self.s = tf.placeholder(tf.float32, [None, N_S], 'S')
            self.target_q = tf.placeholder(tf.float32, [None, 1], 'target_q')
            self._build_net(scope)

            with tf.name_scope('update_c'):
                reg_set_c = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES, scope=scope + '/critic')
                l2_loss_c = tf.add_n(reg_set_c) / BATCH_SIZE
                td = tf.subtract(self.target_q, self.q_value, name='TD_error')
                self.c_loss = tf.reduce_mean(tf.square(td)) + l2_loss_c
                c_grads = tf.gradients(self.c_loss, self.c_params)
                self.update_c = OPT_C.apply_gradients(zip(c_grads, self.c_params))
            with tf.name_scope('update_a'):
                reg_set_a = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES, scope=scope + '/actor')
                l2_loss_a = tf.add_n(reg_set_a) / BATCH_SIZE
                policy_grads = tf.gradients(ys=-self.q_value + l2_loss_a, xs=self.a_params)
                self.update_a = OPT_A.apply_gradients(zip(policy_grads, self.a_params))
                # self.update_a = OPT_A.minimize(-self.q_value + l2_loss_a)


    def _build_net(self, scope):
        """
        Build the Actor Critic network.

        Args:
            scope: Identifier of AC_net.

        Returns:
            action: Output of Actor network.
            q_value: Output of Critic network.
            a_params: Parameters of Actor network.
            c_params: Parameters of Critic network.
        """

        w_init = tf.random_normal_initializer(0, 0.1)
        l2 = tf.contrib.layers.l2_regularizer(1e-7)
        with tf.variable_scope('actor'):
            first_relu = tf.layers.dense(self.s, 2 * N_A, tf.nn.relu,
                                         kernel_initializer=w_init, kernel_regularizer=l2, name='first_relu')
            second_relu = tf.layers.dense(first_relu, 2 * N_A, tf.nn.relu,
                                          kernel_initializer=w_init, kernel_regularizer=l2, name='second_relu')
            self.action = tf.layers.dense(second_relu, N_A,
                                         kernel_initializer=w_init, kernel_regularizer=l2, name='action')
        with tf.variable_scope('critic'):
            state_map = tf.layers.dense(self.s, N_A, tf.nn.relu,
                                        kernel_initializer=w_init, kernel_regularizer=l2, name='state_map')
            critic_input = tf.concat([self.action, state_map], 1)
            first_relu = tf.layers.dense(critic_input, N_A, tf.nn.relu,
                                         kernel_initializer=w_init, kernel_regularizer=l2, name='first_relu')
            second_relu = tf.layers.dense(first_relu, 20, tf.nn.relu,
                                          kernel_initializer=w_init, kernel_regularizer=l2, name='second_relu')
            self.q_value = tf.layers.dense(second_relu, 1, tf.nn.relu,
                                      kernel_initializer=w_init, kernel_regularizer=l2, name='q_value')
        self.a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/actor')
        self.c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/critic')

    def train(self, target_q, s):
        """
        Update AC_net.

        Args:
            target_q: Output of target Critic.
            s: Current state.
        """
        SESS.run([self.update_a, self.update_c], feed_dict={self.s: s,
                                                            self.target_q: target_q})

    def set_params(self, a_params, c_params, t):
        """
        Update target network.

        :param a_params: Train Actor parameters.
        :param c_params: Train Critic parameters.
        :param t: Update weights.
        :return: None
        """
        update_a = [old.assign((1-t)*old + t*new) for old, new in zip(self.a_params, a_params)]
        update_c = [old.assign((1-t)*old + t*new) for old, new in zip(self.c_params, c_params)]

        SESS.run([update_a, update_c])

    def get_action(self, s):
        """
        Get action based on state.

        :param s: State.
        :return: Action.
        """
        s = np.array(s).reshape((1, N_S))
        return SESS.run(self.action, feed_dict={self.s: s})

    def get_q(self, s):
        """
        Get q_value based on state.

        :param s: State.
        :return: Q value.
        """
        return SESS.run(self.q_value, feed_dict={self.s: s})

def init_params(reader):
    v_names = ['train_AC/actor/first_relu/kernel:0',
               'train_AC/actor/first_relu/bias:0',
               'train_AC/actor/second_relu/kernel:0',
               'train_AC/actor/second_relu/bias:0',
               'train_AC/actor/action/kernel:0',
               'train_AC/actor/action/bias:0']
    var_s = tf.trainable_variables()
    for i in range(len(var_s)):
        for name in v_names:
            if var_s[i].name == name:
                SESS.run(var_s[i].assign(reader.get_tensor(name[15:-2])))

class Memory(object):
    """
    Replay memory saving (St, At, St+1 r).

    Attributes:
        capacity: max records numbers.
        data: numpy array saving records.
        pointer: serial number of current records.
    """
    def __init__(self, capacity, dims):
        """
        Initialize the memory.

        :param capacity: max record numbers.
        :param dims: length of one record.
        """
        self.capacity = capacity
        self.data = np.zeros((capacity, dims))
        self.pointer = 0

    def store_transition(self, s, a, r, s_):
        """
        Store one transition.
        If the memory is full, replace the earlist one

        :param s: current state.
        :param a: action based on current state
        :param r: reward based on current state and action.
        :param s_: next state based on current state and action.
        :return: None
        """
        transition = np.hstack((s, np.squeeze(a), [r], s_))
        index = self.pointer % self.capacity  # replace the old memory with new memory
        self.data[index, :] = transition
        self.pointer += 1

    def sample(self, n):
        """
        Sample a batch for training.

        :param n: Batch size.
        :return: Sampled batch.
        """
        # assert self.pointer >= self.capacity, 'Memory has not been fulfilled'
        indices = np.random.choice(self.capacity, size=n)
        sampled_data = self.data[indices, :]
        b_s = sampled_data[:, :N_S]
        b_a = sampled_data[:, N_S: N_S + N_A]
        b_r = sampled_data[:, -N_S - 1: -N_S]
        b_s_ = sampled_data[:, -N_S:]
        return np.array(b_s), np.array(b_a), np.array(b_r), np.array(b_s_)

def gen_res():
    print('start recommend.')
    half_experiences = {}
    recs = {}
    print('\rrecommending for user ')
    for user in tqdm(range(1, env.user_num+1)):
        # print('\rrecommending for user {} / {}'.format(user, env.user_num), end='', flush=True)
        s = env.state[user]
        a = train_AC.get_action(s)
        e = random.random()
        if e < E_GREEDY:
            noise = np.random.randn(1, N_A)
            a = np.add(a, noise)
        half_experiences[user] = [s, a]
        rec_items = env.step(user, a)
        recs[user] = rec_items
    return half_experiences, recs

def feedback_to_experiences(recs, half_experiences, feedback_users):
    for user in half_experiences.keys():
        if user in feedback_users:
            env.user_history[user] += recs[user]
            env.state[user] = env.state_encode(user, env.user_history[user])
            half_experiences[user] += [1, env.state[user]]
        else:
            half_experiences[user] += [0, env.state[user]]
    return half_experiences


if __name__ == '__main__':
    train_AC = AC_net('train_AC')
    target_AC = AC_net('target_AC')
    SESS.run(tf.global_variables_initializer())
    reader = tf.train.NewCheckpointReader('./ckpt/MM.ckpt')
    init_params(reader)
    del reader
    target_AC.set_params(train_AC.a_params, train_AC.c_params, t=1)

    pos_memory = Memory(memory_capacity, 2*N_S + N_A + 1)
    neg_memory = Memory(memory_capacity, 2*N_S + N_A + 1)

    while 1:
        half_experiences, recs = gen_res()
        res_dic_to_csv(recs, user_info_path, item_info_path)
        command = input('Enter command(feedback or break?):')
        while command not in ['feedback', 'break']:
            command = input('Enter command(feedback or break?):')
        if command == 'feedback':
            feedback_users = read_users_to_set('./feedback/hit_users_1m.txt', user_info_path)
        else:
            break
        full_experiences = feedback_to_experiences(recs, half_experiences, feedback_users)

        del half_experiences, recs

        for experience in full_experiences.values():
            if experience[2] == 1:
                pos_memory.store_transition(*experience)
            else:
                neg_memory.store_transition(*experience)
        print('experience stored.')

        del full_experiences

        # 网络训练和参数更新
        b_s, b_a, b_r, b_s_ = pos_memory.sample(BATCH_SIZE)
        b__s, b__a, b__r, b__s_ = neg_memory.sample(len(b_s)*2)
        q_ = target_AC.get_q(np.concatenate([b_s_,b__s_], axis=0))
        target_q = (np.concatenate([b_r, b__r], axis=0)) + GAMMA * q_
        train_AC.train(target_q, s=np.concatenate([b_s, b__s], axis=0))
        target_AC.set_params(train_AC.a_params, train_AC.c_params, t=0.8)
        print('parameter updated.')

        gc.collect()


