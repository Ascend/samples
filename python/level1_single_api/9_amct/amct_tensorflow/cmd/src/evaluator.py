import amct_tensorflow.common.cmd_line_utils.data_handler as data_handler
import tensorflow as tf


class Evaluator():
    def calibration(self, graph, infer_output):
        with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
            sess.run(tf.global_variables_initializer())
            for data_map in data_handler.load_data(
                            {'input:0': [32, 224, 224, 3]},
                            ['./data/image'],
                            ['float32'],
                            1
                            ):
                results = sess.run(infer_output, feed_dict=data_map)

customize_evaluator = Evaluator()