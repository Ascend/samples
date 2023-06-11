# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import tensorflow as tf
from six.moves import xrange
import time
from datetime import datetime
import os
from genotypes import Structure
from config import FLAGS
import cifar
import numpy as np
from npu_bridge.npu_init import *
import infer

TAG = 'NAS_Bench_201:' + os.path.basename(__file__)


def train():
    config = tf.ConfigProto()
    custom_op = config.graph_options.rewrite_options.custom_optimizers.add()
    custom_op.name = "NpuOptimizer"
    custom_op.parameter_map["session_device_id"].i = FLAGS.card_id
    # custom_op.parameter_map["dump_path"].s = tf.compat.as_bytes("dump_data/")
    # # enable_dump_debug：是否开启溢出检测功能
    # custom_op.parameter_map["enable_dump_debug"].b = True
    # # dump_debug_mode：溢出检测模式，取值：all/aicore_overflow/atomic_overflow
    # custom_op.parameter_map["dump_debug_mode"].s = tf.compat.as_bytes("all")
    custom_op.parameter_map["precision_mode"].s = tf.compat.as_bytes("force_fp32")
    config.graph_options.rewrite_options.remapping = RewriterConfig.OFF
    config.graph_options.rewrite_options.memory_optimization = RewriterConfig.OFF

    op = ['nor_conv_1x1', 'nor_conv_3x3', 'avg_pool_3x3', 'skip_connect', 'none']
    for arch_file_id in FLAGS.training_arch_list:
        file_name = FLAGS.arch_file_path + 'arch' + str(arch_file_id) + '.txt'
        train_info_list = []
        for line in open(file_name, 'r'):
            line = line.replace('\n', '')
            train_info_list.append(line)

        for idx, info in enumerate(train_info_list):
            file = open(FLAGS.result_file_path + 'result_' + str(arch_file_id) + '.txt', 'a')
            train_info = info.strip().split(',')
            code_str = train_info[0]
            code_info = train_info[1]
            if code_info == 'False':
                code = list(map(int, list(code_str)))
                print('arch_code:%s' % (str(code_str)))
                genotype = Structure(
                    [
                        ((op[code[0]], 0),),
                        ((op[code[1]], 0), (op[code[2]], 1)),
                        ((op[code[3]], 0), (op[code[4]], 1), (op[code[5]], 2))
                    ]
                )
                time.sleep(10)
                tf.reset_default_graph()
                with tf.Graph().as_default():
                    global_step = tf.train.get_or_create_global_step()

                    images, labels = cifar.CIFARInput('train').input_fn()

                    image_holder = tf.placeholder(tf.float32, [FLAGS.batch_size, 32, 32, 3])
                    label_holder = tf.placeholder(tf.int32, [FLAGS.batch_size])

                    logits = infer.inference(image_holder, genotype, C=16, N=5)
                    print(TAG, logits)

                    loss = infer.loss_fun(logits, label_holder)
                    print(TAG, loss)

                    train_op = infer.train(loss, global_step)
                    print(TAG, train_op)

                    top_k_op = tf.nn.in_top_k(logits, label_holder, 1)

                    saver = tf.train.Saver(tf.all_variables())

                    init = tf.initialize_all_variables()

                    sess = tf.Session(config=config)
                    sess.run(init)

                    tf.train.start_queue_runners(sess=sess)
                    total_time_start = time.time()
                    best_acc = -1.0
                    for step in xrange(FLAGS.max_steps):
                        start_time = time.time()
                        image_batch, label_batch = sess.run([images, labels])
                        _, loss_value = sess.run([train_op, loss], feed_dict={image_holder: image_batch,
                                                                              label_holder: label_batch})
                        duration = time.time() - start_time

                        num_examples_per_step = FLAGS.batch_size
                        examples_per_sec = num_examples_per_step / duration
                        sec_per_batch = float(duration)

                        # test
                        if step % 194 == 0:  # one epoch contains 194 steps
                            format_str = ('%s: step %d, loss = %.2f (%.1f examples/sec; %.3f '
                                          'sec/batch)')
                            print(TAG, format_str % (datetime.now(), step, loss_value, examples_per_sec, sec_per_batch))
                            num_examples = 10000
                            num_iter = int(num_examples / FLAGS.batch_size)
                            true_count = 0
                            total_sample_count = num_iter * FLAGS.batch_size
                            images_test, labels_test = cifar.CIFARInput('valid').input_fn()
                            for test_step in xrange(num_iter):
                                image_batch_, label_batch_ = sess.run([images_test, labels_test])
                                predictions = sess.run([top_k_op], feed_dict={image_holder: image_batch_,
                                                                              label_holder: label_batch_})
                                true_count += np.sum(predictions)
                            print(TAG, 'true_count', true_count)
                            precision = true_count / total_sample_count
                            if precision > best_acc:
                                best_acc = precision
                                print(TAG, 'best_acc= %.3f' % best_acc)
                            print(TAG, 'precision= %.3f' % precision)
                            # checkpoint_path = os.path.join(FLAGS.train_dir, 'model.ckpt')
                            # saver.save(sess, checkpoint_path, global_step=step)
                    total_duration = time.time() - total_time_start
                    total_sec_time = float(total_duration)

                    format_str = ('id:%d, code:%s, total_time:%.3f, best_acc:%.3f')
                    file.write(format_str % (idx, str(code), total_sec_time, best_acc) + '\n')
                    file.close()
                    code_info = 'True'
                    infer.alter(file_name, info, code_str + ',' + code_info)
                    format_str = ('%s: code:%s, total_time = %.3f, best_acc = %.3f')
                    print(TAG, format_str % (datetime.now(), str(code), total_sec_time, best_acc))
            else:
                print(TAG, code_str, 'already trained!!!')


def main(argv=None):
    train()


if __name__ == '__main__':
    tf.app.run()