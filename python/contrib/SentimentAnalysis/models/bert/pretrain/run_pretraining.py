# coding=utf-8
# Copyright (c) 2019 NVIDIA CORPORATION. All rights reserved.
# Copyright 2018 The Google AI Language Team Authors.
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
"""Run masked LM/next sentence masked_lm pre-training for BERT."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
from bert.pretrain import modeling
from bert.pretrain import optimization
import tensorflow as tf
import glob
from bert.pretrain.utils import LogEvalRunHook
from tensorflow.core.protobuf import rewriter_config_pb2
from bert.pretrain.gpu_environment import get_custom_getter

# from npu_bridge.estimator.npu.npu_config import *
# from npu_bridge.estimator.npu.npu_estimator import *
# from npu_bridge.estimator.npu.npu_config import NPURunConfig
# from npu_bridge.estimator.npu.npu_estimator import NPUEstimator

os.environ['WHICH_OP'] = 'GEOP'
os.environ['NEW_GE_FE_ID'] = '1'
os.environ['GE_AICPU_FLAG'] = '1'
os.environ['GE_USE_STATIC_MEMORY'] = '1'
os.environ['OPTION_EXEC_HCCL_FLAG'] = '1'
os.environ['HCCL_CONNECT_TIMEOUT'] = '600'

flags = tf.flags

FLAGS = flags.FLAGS

## Required parameters
flags.DEFINE_string(
    "bert_config_file", None,
    "The config json file corresponding to the pre-trained BERT model. "
    "This specifies the model architecture.")

flags.DEFINE_string(
    "input_files_dir", "./data",
    "Directory with input files, comma separated or single directory.")

flags.DEFINE_string(
    "eval_files_dir", None,
    "Directory with eval files, comma separated or single directory. ")

flags.DEFINE_string(
    "output_dir", "./models",
    "The output directory where the model checkpoints will be written.")

## Other parameters
flags.DEFINE_string(
    "init_checkpoint", None,
    "Initial checkpoint (usually from a pre-trained BERT model).")

flags.DEFINE_string("optimizer_type", "lamb",
                    "Optimizer used for training - LAMB or ADAM")

flags.DEFINE_integer(
    "max_seq_length", 128,
    "The maximum total input sequence length after WordPiece tokenization. "
    "Sequences longer than this will be truncated, and sequences shorter "
    "than this will be padded. Must match data generation.")

flags.DEFINE_integer(
    "max_predictions_per_seq", 20,
    "Maximum number of masked LM predictions per sequence. "
    "Must match data generation.")

flags.DEFINE_bool("do_train", True, "Whether to run training.")

flags.DEFINE_bool("do_eval", False, "Whether to run eval on the dev set.")

flags.DEFINE_integer("train_batch_size", 64, "Total batch size for training.")

flags.DEFINE_integer("eval_batch_size", 8, "Total batch size for eval.")

flags.DEFINE_float("learning_rate", 1e-4,
                   "The initial learning rate for Adam.")

flags.DEFINE_integer("num_train_steps", 1000000, "Number of training steps.")

flags.DEFINE_integer("num_warmup_steps", 10000, "Number of warmup steps.")

flags.DEFINE_integer("save_checkpoints_steps", 10000,
                     "How often to save the model checkpoint.")

flags.DEFINE_integer("display_loss_steps", 10, "How often to print loss")

flags.DEFINE_integer("iterations_per_loop", 1000,
                     "How many steps to make in each estimator call.")

flags.DEFINE_integer("max_eval_steps", 100, "Maximum number of eval steps.")

flags.DEFINE_integer(
    "num_accumulation_steps", 1,
    "Number of accumulation steps before gradient update."
    "Global batch size = num_accumulation_steps * train_batch_size")

flags.DEFINE_bool(
    "allreduce_post_accumulation", False,
    "Whether to all reduce after accumulation of N steps or after each step")

flags.DEFINE_bool("verbose_logging", False,
                  "If true, all of the trainable parameters are printed")

flags.DEFINE_bool("horovod", False,
                  "Whether to use Horovod for multi-gpu runs")

flags.DEFINE_bool("report_loss", True,
                  "Whether to report total loss during training.")

flags.DEFINE_bool(
    "manual_fp16", True, "Whether to use fp32 or fp16 arithmetic on GPU. "
    "Manual casting is done instead of using AMP")

flags.DEFINE_bool("use_xla", False, "Whether to enable XLA JIT compilation.")

flags.DEFINE_bool("use_fp16", False, "Whether to enable AMP ops.")

flags.DEFINE_bool("use_fp16_cls", True,
                  "Whether to use fp16 in cls and pooler.")

flags.DEFINE_bool("distributed", True, "Whether to use multi-npu")

flags.DEFINE_bool('npu_bert_fused_gelu', True,
                  'Whether to use npu defined gelu op')

flags.DEFINE_bool('npu_bert_debug', False,
                  'If True, dropout and shuffle is disabled.')

flags.DEFINE_bool('npu_bert_use_tdt', True, 'Whether to use tdt as dataset')

flags.DEFINE_string("npu_bert_job_start_file", None,
                    "CSA job start file path.")

flags.DEFINE_integer(
    "npu_bert_loss_scale", 0,
    "Whether to use loss scale, -1 is disable, 0 is dynamic loss scale, >=1 is static loss scale"
)

flags.DEFINE_bool(
    "npu_bert_clip_by_global_norm", False,
    "Use clip_by_global_norm if True, or use clip_by_norm for each gradient")

flags.DEFINE_bool('npu_bert_npu_dropout', True,
                  'Whether to use npu defined gelu op')

flags.DEFINE_bool(
    'npu_gather', True,
    'Whether to use gather_npu whose backward propagation avoids IndexedSlices'
)

flags.DEFINE_bool('hcom_parallel', True, 'Whether to use parallel allreduce')

flags.DEFINE_integer('init_loss_scale_value', 2 ** 32,
                     'Initial loss scale value for loss scale optimizer')


# report samples/sec, total loss and learning rate during training
class _LogSessionRunHook(tf.train.SessionRunHook):
    def __init__(self,
                 global_batch_size,
                 num_accumulation_steps,
                 display_every=10,
                 hvd_rank=-1):
        self.global_batch_size = global_batch_size
        self.display_every = display_every
        self.hvd_rank = hvd_rank
        self.num_accumulation_steps = num_accumulation_steps
        self.count = 0
        self.t0 = time.time()
        self.elapsed_secs = 0.
        self.count = 0
        self.all_count = 0
        self.avg_loss = 0.0

    def after_create_session(self):
        """
        after create session, reset env.
        """
        self.elapsed_secs = 0.
        self.count = 0
        self.all_count = 0
        self.avg_loss = 0.0

    def before_run(self):
        """
        before run, set env.
        """
        self.t0 = time.time()
        if self.num_accumulation_steps <= 1:
            if (tf.flags.FLAGS.npu_bert_loss_scale
                    == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                return tf.train.SessionRunArgs(fetches=[
                    'global_step:0', 'total_loss:0', 'learning_rate:0',
                    'nsp_loss:0', 'mlm_loss:0', 'loss_scale:0',
                    'apply_grads/All:0'
                ])
            else:
                return tf.train.SessionRunArgs(fetches=[
                    'global_step:0', 'total_loss:0', 'learning_rate:0',
                    'nsp_loss:0', 'mlm_loss:0'
                ])
        else:
            if (tf.flags.FLAGS.npu_bert_loss_scale
                    == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                return tf.train.SessionRunArgs(fetches=[
                    'global_step:0', 'update_step:0', 'total_loss:0',
                    'learning_rate:0', 'nsp_loss:0', 'mlm_loss:0',
                    'loss_scale:0'
                ])
            else:
                return tf.train.SessionRunArgs(fetches=[
                    'global_step:0', 'update_step:0', 'total_loss:0',
                    'learning_rate:0', 'nsp_loss:0', 'mlm_loss:0'
                ])

    def after_run(self, run_values):
        """
        after run, do same upate.
        Args:
            run_values (list): values of outputs
        """
        self.elapsed_secs += time.time() - self.t0
        if self.num_accumulation_steps <= 1:
            if (tf.flags.FLAGS.npu_bert_loss_scale
                    == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                global_step, total_loss, lr, nsp_loss, mlm_loss, loss_scaler, custom_arg = run_values.results
            else:
                global_step, total_loss, lr, nsp_loss, mlm_loss = run_values. \
                    results
            update_step = True
        else:
            if (tf.flags.FLAGS.npu_bert_loss_scale
                    == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                global_step, update_step, total_loss, lr, nsp_loss, mlm_loss, loss_scaler = run_values.results
            else:
                global_step, update_step, total_loss, lr, nsp_loss, mlm_loss = run_values.\
                    results
        print_step = global_step + 1  # One-based index for printing.
        self.avg_loss += total_loss
        self.all_count += 1
        if update_step:
            self.count += 1
            dt = self.elapsed_secs / self.count
            sent_per_sec = self.global_batch_size / dt * FLAGS.iterations_per_loop
            avg_loss_step = self.avg_loss / self.all_count
            if self.hvd_rank >= 0:
                if (tf.flags.FLAGS.npu_bert_loss_scale
                        == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                    print(
                        'Rank = %2d :: Step = %6i Throughput = %11.1f MLM Loss = %10.4e NSP Loss = %10.4e Loss = %9.6f Average Loss = %9.6f LR = %6.4e Loss scale = %6.4e isFinite = %6i'
                        % (self.hvd_rank, print_step, sent_per_sec, mlm_loss,
                           nsp_loss, total_loss, avg_loss_step, lr,
                           loss_scaler, custom_arg),
                        flush=True)
                else:
                    print(
                        'Rank = %2d :: Step = %6i Throughput = %11.1f MLM Loss = %10.4e NSP Loss = %10.4e Loss = %9.6f Average Loss = %9.6f LR = %6.4e'
                        % (self.hvd_rank, print_step, sent_per_sec, mlm_loss,
                           nsp_loss, total_loss, avg_loss_step, lr),
                        flush=True)
            else:
                if (tf.flags.FLAGS.npu_bert_loss_scale
                        == 0) and (FLAGS.manual_fp16 or FLAGS.use_fp16):
                    print(
                        'Step = %6i Throughput = %11.1f MLM Loss = %10.4e NSP Loss = %10.4e Loss = %9.6f Average Loss = %9.6f LR = %6.4e Loss scale = %6.4e isFinite = %6i'
                        % (print_step, sent_per_sec, mlm_loss, nsp_loss,
                           total_loss, avg_loss_step, lr, loss_scaler,
                           custom_arg),
                        flush=True)
                else:
                    print(
                        'Step = %6i Throughput = %11.1f MLM Loss = %10.4e NSP Loss = %10.4e Loss = %9.6f Average Loss = %9.6f LR = %6.4e'
                        % (print_step, sent_per_sec, mlm_loss, nsp_loss,
                           total_loss, avg_loss_step, lr),
                        flush=True)
            self.elapsed_secs = 0.
            self.count = 0
            self.avg_loss = 0.0
            self.all_count = 0


def model_fn_builder(bert_config,
                     init_checkpoint,
                     learning_rate,
                     num_train_steps,
                     num_warmup_steps,
                     use_one_hot_embeddings,
                     hvd=None):
    """Returns `model_fn` closure for TPUEstimator."""
    def model_fn(features, labels, mode, params):  # pylint: disable=unused-argument
        """The `model_fn` for TPUEstimator."""

        tf.logging.info("*** Features ***")
        for name in sorted(features.keys()):
            tf.logging.info("  name = %s, shape = %s" %
                            (name, features[name].shape))

        input_ids = features["input_ids"]
        input_mask = features["input_mask"]
        segment_ids = features["segment_ids"]
        masked_lm_positions = features["masked_lm_positions"]
        masked_lm_ids = features["masked_lm_ids"]
        masked_lm_weights = features["masked_lm_weights"]
        next_sentence_labels = features["next_sentence_labels"]

        is_training = (mode == tf.estimator.ModeKeys.TRAIN)

        model = modeling.BertModel(
            config=bert_config,
            is_training=is_training,
            input_ids=input_ids,
            input_mask=input_mask,
            token_type_ids=segment_ids,
            use_one_hot_embeddings=use_one_hot_embeddings,
            compute_type=tf.float16 if FLAGS.manual_fp16 else tf.float32)

        (masked_lm_loss, masked_lm_example_loss,
         masked_lm_log_probs) = get_masked_lm_output(
             bert_config, model.get_sequence_output(),
             model.get_embedding_table(), masked_lm_positions, masked_lm_ids,
             masked_lm_weights)

        (next_sentence_loss, next_sentence_example_loss,
         next_sentence_log_probs) = get_next_sentence_output(
             bert_config, model.get_pooled_output(), next_sentence_labels)

        masked_lm_loss = tf.identity(masked_lm_loss, name="mlm_loss")
        next_sentence_loss = tf.identity(next_sentence_loss, name="nsp_loss")
        total_loss = masked_lm_loss + next_sentence_loss
        total_loss = tf.identity(total_loss, name='total_loss')

        tvars = tf.trainable_variables()

        initialized_variable_names = {}
        if init_checkpoint and (hvd is None or hvd.rank() == 0):
            print("Loading checkpoint", init_checkpoint)
            (assignment_map, initialized_variable_names
             ) = modeling.get_assignment_map_from_checkpoint(
                 tvars, init_checkpoint)

            tf.train.init_from_checkpoint(init_checkpoint, assignment_map)

        if FLAGS.verbose_logging:
            tf.logging.info("**** Trainable Variables ****")
            for var in tvars:
                init_string = ""
                if var.name in initialized_variable_names:
                    init_string = ", *INIT_FROM_CKPT*"
                tf.logging.info("  %d :: name = %s, shape = %s%s",
                                0 if hvd is None else hvd.rank(), var.name,
                                var.shape, init_string)

        output_spec = None
        if mode == tf.estimator.ModeKeys.TRAIN:
            train_op = optimization.create_optimizer(
                total_loss, learning_rate, num_train_steps, num_warmup_steps,
                hvd, FLAGS.manual_fp16, FLAGS.use_fp16,
                FLAGS.num_accumulation_steps, FLAGS.optimizer_type,
                FLAGS.allreduce_post_accumulation)

            output_spec = tf.estimator.EstimatorSpec(mode=mode,
                                                     loss=total_loss,
                                                     train_op=train_op)
        elif mode == tf.estimator.ModeKeys.EVAL:

            def metric_fn(masked_lm_example_loss, masked_lm_log_probs,
                          masked_lm_ids, masked_lm_weights,
                          next_sentence_example_loss, next_sentence_log_probs,
                          next_sentence_labels):
                """Computes the loss and accuracy of the model."""
                masked_lm_log_probs = tf.reshape(
                    masked_lm_log_probs, [-1, masked_lm_log_probs.shape[-1]])
                masked_lm_predictions = tf.argmax(masked_lm_log_probs,
                                                  axis=-1,
                                                  output_type=tf.int32)
                masked_lm_example_loss = tf.reshape(masked_lm_example_loss,
                                                    [-1])
                masked_lm_ids = tf.reshape(masked_lm_ids, [-1])
                masked_lm_weights = tf.reshape(masked_lm_weights, [-1])
                masked_lm_accuracy = tf.metrics.accuracy(
                    labels=masked_lm_ids,
                    predictions=masked_lm_predictions,
                    weights=masked_lm_weights)
                masked_lm_mean_loss = tf.metrics.mean(
                    values=masked_lm_example_loss, weights=masked_lm_weights)

                next_sentence_log_probs = tf.reshape(
                    next_sentence_log_probs,
                    [-1, next_sentence_log_probs.shape[-1]])
                next_sentence_predictions = tf.argmax(next_sentence_log_probs,
                                                      axis=-1,
                                                      output_type=tf.int32)
                next_sentence_labels = tf.reshape(next_sentence_labels, [-1])
                next_sentence_accuracy = tf.metrics.accuracy(
                    labels=next_sentence_labels,
                    predictions=next_sentence_predictions)
                next_sentence_mean_loss = tf.metrics.mean(
                    values=next_sentence_example_loss)

                return {
                    "masked_lm_accuracy": masked_lm_accuracy,
                    "masked_lm_loss": masked_lm_mean_loss,
                    "next_sentence_accuracy": next_sentence_accuracy,
                    "next_sentence_loss": next_sentence_mean_loss,
                }

            eval_metric_ops = metric_fn(masked_lm_example_loss,
                                        masked_lm_log_probs, masked_lm_ids,
                                        masked_lm_weights,
                                        next_sentence_example_loss,
                                        next_sentence_log_probs,
                                        next_sentence_labels)
            output_spec = tf.estimator.EstimatorSpec(
                mode=mode, loss=total_loss, eval_metric_ops=eval_metric_ops)
        else:
            raise ValueError("Only TRAIN and EVAL modes are supported: %s" %
                             (mode))

        return output_spec

    return model_fn


def get_masked_lm_output(bert_config, input_tensor, output_weights, positions,
                         label_ids, label_weights):
    """Get loss and log probs for the masked LM."""
    input_tensor = gather_indexes(input_tensor, positions)

    with tf.variable_scope("cls/predictions"):
        # We apply one more non-linear transformation before the output layer.
        # This matrix is not used after pre-training.
        with tf.variable_scope("transform",
                               custom_getter=get_custom_getter(
                                   compute_type=tf.float16 if FLAGS.
                                   use_fp16_cls else tf.float32)):
            if FLAGS.use_fp16_cls:
                input_tensor = tf.cast(input_tensor, tf.float16)
            input_tensor = tf.layers.dense(
                input_tensor,
                units=bert_config.hidden_size,
                activation=modeling.get_activation(bert_config.hidden_act),
                kernel_initializer=modeling.create_initializer(
                    bert_config.initializer_range))
            input_tensor = tf.cast(input_tensor, tf.float32)
            input_tensor = modeling.layer_norm(input_tensor)

        # The output weights are the same as the input embeddings, but there is
        # an output-only bias for each token.
        output_bias = tf.get_variable("output_bias",
                                      shape=[bert_config.vocab_size],
                                      initializer=tf.zeros_initializer())
        if FLAGS.use_fp16_cls:
            input_tensor = tf.cast(input_tensor, tf.float16)
            logits = tf.matmul(input_tensor,
                               tf.cast(output_weights, tf.float16),
                               transpose_b=True)
            logits = tf.cast(logits, tf.float32)
        else:
            logits = tf.matmul(tf.cast(input_tensor, tf.float32),
                               output_weights,
                               transpose_b=True)
        logits = tf.nn.bias_add(logits, output_bias)
        log_probs = tf.nn.log_softmax(logits, axis=-1)

        label_ids = tf.reshape(label_ids, [-1])
        label_weights = tf.reshape(label_weights, [-1])

        one_hot_labels = tf.one_hot(label_ids,
                                    depth=bert_config.vocab_size,
                                    dtype=tf.float32)

        # The `positions` tensor might be zero-padded (if the sequence is too
        # short to have the maximum number of predictions). The `label_weights`
        # tensor has a value of 1.0 for every real prediction and 0.0 for the
        # padding predictions.
        per_example_loss = -tf.reduce_sum(log_probs * one_hot_labels,
                                          axis=[-1])
        numerator = tf.reduce_sum(label_weights * per_example_loss)
        denominator = tf.reduce_sum(label_weights) + 1e-5
        loss = numerator / denominator

    return (loss, per_example_loss, log_probs)


def get_next_sentence_output(bert_config, input_tensor, labels):
    """Get loss and log probs for the next sentence prediction."""

    # Simple binary classification. Note that 0 is "next sentence" and 1 is
    # "random sentence". This weight matrix is not used after pre-training.
    with tf.variable_scope("cls/seq_relationship"):
        output_weights = tf.get_variable(
            "output_weights",
            shape=[2, bert_config.hidden_size],
            initializer=modeling.create_initializer(
                bert_config.initializer_range))
        output_bias = tf.get_variable("output_bias",
                                      shape=[2],
                                      initializer=tf.zeros_initializer())

        if FLAGS.use_fp16_cls:
            input_tensor = tf.cast(input_tensor, tf.float16)
            logits = tf.matmul(input_tensor,
                               tf.cast(output_weights, tf.float16),
                               transpose_b=True)
            logits = tf.cast(logits, tf.float32)
        else:
            logits = tf.matmul(tf.cast(input_tensor, tf.float32),
                               output_weights,
                               transpose_b=True)
        logits = tf.nn.bias_add(logits, output_bias)
        log_probs = tf.nn.log_softmax(logits, axis=-1)
        labels = tf.reshape(labels, [-1])
        one_hot_labels = tf.one_hot(labels, depth=2, dtype=tf.float32)
        per_example_loss = -tf.reduce_sum(one_hot_labels * log_probs, axis=-1)
        loss = tf.reduce_mean(per_example_loss)
        return (loss, per_example_loss, log_probs)


def gather_indexes(sequence_tensor, positions):
    """Gathers the vectors at the specific positions over a minibatch."""
    sequence_shape = modeling.get_shape_list(sequence_tensor, expected_rank=3)
    batch_size = sequence_shape[0]
    seq_length = sequence_shape[1]
    width = sequence_shape[2]

    flat_offsets = tf.reshape(
        tf.range(0, batch_size, dtype=tf.int32) * seq_length, [-1, 1])
    flat_positions = tf.reshape(positions + flat_offsets, [-1])
    flat_sequence_tensor = tf.reshape(sequence_tensor,
                                      [batch_size * seq_length, width])
    output_tensor = tf.gather(flat_sequence_tensor, flat_positions)
    return output_tensor


def input_fn_builder(input_files,
                     batch_size,
                     max_seq_length,
                     max_predictions_per_seq,
                     is_training,
                     num_cpu_threads=4):
    """Creates an `input_fn` closure to be passed to Estimator."""
    def input_fn():
        """The actual input function."""
        name_to_features = {
            "input_ids":
            tf.FixedLenFeature([max_seq_length], tf.int64),
            "input_mask":
            tf.FixedLenFeature([max_seq_length], tf.int64),
            "segment_ids":
            tf.FixedLenFeature([max_seq_length], tf.int64),
            "masked_lm_positions":
            tf.FixedLenFeature([max_predictions_per_seq], tf.int64),
            "masked_lm_ids":
            tf.FixedLenFeature([max_predictions_per_seq], tf.int64),
            "masked_lm_weights":
            tf.FixedLenFeature([max_predictions_per_seq], tf.float32),
            "next_sentence_labels":
            tf.FixedLenFeature([1], tf.int64),
        }

        # For training, we want a lot of parallel reading and shuffling.
        # For eval, we want no shuffling and parallel reading doesn't matter.
        if is_training:
            d = tf.data.Dataset.from_tensor_slices(tf.constant(input_files))
            if FLAGS.distributed:
                rank_size = int(os.getenv('RANK_SIZE'))
                rank_id = int(os.getenv('RANK_INDEX'))
                device_id = int(os.getenv('DEVICE_ID'))
                local_rank = rank_id * 8 + device_id
                print('RANK_SIZE=', rank_size, ' RANK_ID=', local_rank)
                d = d.shard(rank_size, local_rank)
            d = d.repeat()
            if not FLAGS.npu_bert_debug:
                d = d.shuffle(buffer_size=len(input_files))

            # `cycle_length` is the number of parallel files that get read.
            if not FLAGS.npu_bert_debug:
                #cycle_length = min(num_cpu_threads, len(input_files))
                cycle_length = min(
                    num_cpu_threads,
                    int(len(input_files) / int(os.getenv('RANK_SIZE'))))
            else:
                cycle_length = 1

            # `sloppy` mode means that the interleaving is not exact. This adds
            # even more randomness to the training pipeline.
            #d = d.apply(
            #    tf.contrib.data.parallel_interleave(
            #        tf.data.TFRecordDataset,
            #        sloppy=(not FLAGS.npu_bert_debug),
            #        cycle_length=cycle_length))
            d = d.interleave(tf.data.TFRecordDataset,
                             cycle_length=cycle_length,
                             num_parallel_calls=tf.data.experimental.AUTOTUNE)
            if not FLAGS.npu_bert_debug:
                d = d.shuffle(buffer_size=100)
        else:
            d = tf.data.TFRecordDataset(input_files)
            # Since we evaluate for a fixed number of steps we don't want to encounter
            # out-of-range exceptions.
            d = d.repeat()

        # We must `drop_remainder` on training because the TPU requires fixed
        # size dimensions. For eval, we assume we are evaluating on the CPU or GPU
        # and we *don't* want to drop the remainder, otherwise we wont cover
        # every sample.
        d = d.apply(
            tf.contrib.data.map_and_batch(
                lambda record: _decode_record(record, name_to_features),
                batch_size=batch_size,
                num_parallel_batches=num_cpu_threads,
                drop_remainder=True))
        return d

    return input_fn


def _decode_record(record, name_to_features):
    """Decodes a record to a TensorFlow example."""
    example = tf.parse_single_example(record, name_to_features)

    # tf.Example only supports tf.int64, but the TPU only supports tf.int32.
    # So cast all int64 to int32.
    for name in list(example.keys()):
        t = example[name]
        if t.dtype == tf.int64:
            t = tf.to_int32(t)
        example[name] = t

    return example


def main(_):
    """
    main function
    """
    # for name, value in FLAGS.__flags.items():
    #     print("name:", name, "      ", FLAGS[name].value)

    tf.logging.set_verbosity(tf.logging.INFO)

    if not FLAGS.do_train and not FLAGS.do_eval:
        raise ValueError(
            "At least one of `do_train` or `do_eval` must be True.")

    if FLAGS.use_fp16:
        os.environ["TF_ENABLE_AUTO_MIXED_PRECISION_GRAPH_REWRITE"] = "1"

    if FLAGS.horovod:
        import horovod.tensorflow as hvd
        hvd.init()

    bert_config = modeling.BertConfig.from_json_file(FLAGS.bert_config_file)

    # if FLAGS.npu_gather:
    #   if FLAGS.distributed and bert_config.num_hidden_layers == 24:
    #     #from hccl.split.api import set_split_strategy_by_idx
    #     from hccl.split.api import set_split_strategy_by_size
    #     #set_split_strategy_by_idx([8,72,136,200,264,328,392,397])
    #     set_split_strategy_by_size([10,10,10,10,15,15,15,15])
    #   if FLAGS.distributed and bert_config.num_hidden_layers == 12:
    #     from hccl.split.api import set_split_strategy_by_idx
    #     set_split_strategy_by_idx([8,56,104,152,200,205])
    #   if FLAGS.distributed and bert_config.num_hidden_layers == 6:
    #     from hccl.split.api import set_split_strategy_by_idx
    #     set_split_strategy_by_idx([8,40,72,104,109])

    tf.gfile.MakeDirs(FLAGS.output_dir)

    input_files = []
    for input_file_dir in FLAGS.input_files_dir.split(","):
        input_files.extend(tf.gfile.Glob(os.path.join(input_file_dir, "*")))

    input_files.sort()
    print("Input Files:", input_files)

    if FLAGS.horovod and len(input_files) < hvd.size():
        raise ValueError("Input Files must be sharded")
    if FLAGS.use_fp16 and FLAGS.manual_fp16:
        raise ValueError(
            "AMP and Manual Mixed Precision Training are both activated! Error"
        )

    is_per_host = tf.contrib.tpu.InputPipelineConfig.PER_HOST_V2
    config = tf.ConfigProto()
    if FLAGS.horovod:
        config.gpu_options.visible_device_list = str(hvd.local_rank())
        if hvd.rank() == 0:
            tf.logging.info("***** Configuaration *****")
            # for key in FLAGS.__flags.keys():
            #     tf.logging.info('  {}: {}'.format(key, getattr(FLAGS, key)))
            tf.logging.info("**************************")


#    config.gpu_options.per_process_gpu_memory_fraction = 0.7
    if FLAGS.use_xla:
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        config.graph_options.rewrite_options.memory_optimization = rewriter_config_pb2.RewriterConfig.NO_MEM_OPT

    if FLAGS.distributed:
        rank_size = int(os.getenv('RANK_SIZE'))
    model_fn = model_fn_builder(bert_config=bert_config,
                                init_checkpoint=FLAGS.init_checkpoint,
                                learning_rate=FLAGS.learning_rate,
                                num_train_steps=FLAGS.num_train_steps,
                                num_warmup_steps=FLAGS.num_warmup_steps,
                                use_one_hot_embeddings=False,
                                hvd=None if not FLAGS.horovod else hvd)
    
    training_hooks = []
    
    if FLAGS.report_loss:
        global_batch_size = FLAGS.train_batch_size * FLAGS.num_accumulation_steps \
            if not FLAGS.distributed else FLAGS.train_batch_size * FLAGS.num_accumulation_steps * rank_size
        training_hooks.append(
            _LogSessionRunHook(global_batch_size, FLAGS.num_accumulation_steps,
                               FLAGS.display_loss_steps))

    if FLAGS.do_train:
        tf.logging.info("***** Running training *****")
        tf.logging.info("  Batch size = %d", FLAGS.train_batch_size)
        train_input_fn = input_fn_builder(
            input_files=input_files,
            batch_size=FLAGS.train_batch_size,
            max_seq_length=FLAGS.max_seq_length,
            max_predictions_per_seq=FLAGS.max_predictions_per_seq,
            is_training=True) # ,
            # hvd=None if not FLAGS.horovod else hvd)

        #estimator.train(input_fn=train_input_fn, hooks=training_hooks, max_steps=FLAGS.num_train_steps)

    if FLAGS.do_eval and (not FLAGS.horovod or hvd.rank() == 0):
        tf.logging.info("***** Running evaluation *****")
        tf.logging.info("  Batch size = %d", FLAGS.eval_batch_size)

        eval_files = []
        for eval_file_dir in FLAGS.eval_files_dir.split(","):
            eval_files.extend(tf.gfile.Glob(os.path.join(eval_file_dir, "*")))

        eval_input_fn = input_fn_builder(
            input_files=eval_files,
            batch_size=FLAGS.eval_batch_size,
            max_seq_length=FLAGS.max_seq_length,
            max_predictions_per_seq=FLAGS.max_predictions_per_seq,
            is_training=False) # ,
            # hvd=None if not FLAGS.horovod else hvd)

        eval_hooks = [LogEvalRunHook(FLAGS.eval_batch_size)]
        eval_start_time = time.time()
        # result = estimator.evaluate(
        #     input_fn=eval_input_fn, steps=FLAGS.max_eval_steps, hooks=eval_hooks)

        eval_time_elapsed = time.time() - eval_start_time
        eval_time_wo_overhead = eval_hooks[-1].total_time

        num_sentences = (eval_hooks[-1].count -
                         eval_hooks[-1].skipped) * FLAGS.eval_batch_size

        ss_sentences_per_second = num_sentences * 1.0 / eval_time_wo_overhead

        tf.logging.info("-----------------------------")
        tf.logging.info("Total Inference Time = %0.2f for Sentences = %d",
                        eval_time_elapsed,
                        eval_hooks[-1].count * FLAGS.eval_batch_size)
        tf.logging.info(
            "Total Inference Time W/O Overhead = %0.2f for Sentences = %d",
            eval_time_wo_overhead,
            (eval_hooks[-1].count - eval_hooks[-1].skipped) *
            FLAGS.eval_batch_size)
        tf.logging.info("Summary Inference Statistics on EVAL set")
        tf.logging.info("Batch size = %d", FLAGS.eval_batch_size)
        tf.logging.info("Sequence Length = %d", FLAGS.max_seq_length)
        tf.logging.info("Precision = %s", "fp16" if FLAGS.use_fp16 else "fp32")
        tf.logging.info("Throughput Average (sentences/sec) = %0.2f",
                        ss_sentences_per_second)
        tf.logging.info("-----------------------------")

        #output_eval_file = os.path.join(FLAGS.output_dir, "eval_results.txt")
        # with tf.gfile.GFile(output_eval_file, "w") as writer:
        #   tf.logging.info("***** Eval results *****")
        #   for key in sorted(result.keys()):
        #     tf.logging.info("  %s = %s", key, str(result[key]))
        #     writer.write("%s = %s\n" % (key, str(result[key])))

if __name__ == "__main__":
    flags.mark_flag_as_required("input_files_dir")
    flags.mark_flag_as_required("eval_files_dir")
    flags.mark_flag_as_required("bert_config_file")
    flags.mark_flag_as_required("output_dir")
    flags.mark_flag_as_required("npu_bert_job_start_file")
    if FLAGS.use_xla and FLAGS.manual_fp16:
        print(
            'WARNING! Combining --use_xla with --manual_fp16 may prevent convergence.'
        )
        print(
            '         This warning message will be removed when the underlying'
        )
        print(
            '         issues have been fixed and you are running a TF version')
        print('         that has that fix.')
    tf.app.run()
