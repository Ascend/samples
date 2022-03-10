"""
Calculate the data.
"""
import tensorflow as tf
import numpy as np
import sys

def compute(input_file, filter_file, out_file):
    """
Calculate the data.
:param val: input_file, filter_file, out_file.
:type val: Any
:return:print result
"""
    # NHWC
    a = np.fromfile(input_file, dtype=np.float16).reshape([2, 1024, 1024, 3])#.transpose((0, 2, 3, 1))
    #NCHW->HWCN
    b = np.fromfile(filter_file, dtype=np.float16).reshape([6, 3, 3, 3]).transpose((2, 3, 1, 0)) 

    input = tf.Variable(a)
    filter = tf.Variable(b)       

    op = tf.nn.conv2d(input, filter, strides=[1, 1, 1, 1], padding="SAME", 
    use_cudnn_on_gpu=False, data_format='NHWC', dilations=[1, 1, 1, 1], name=None, filters=None)

    sess = tf.Session()
    tf.global_variables_initializer().run(session=sess)
    op_output, input, filter = sess.run([op, input, filter])
    print(op_output.shape)
    c = np.fromfile(out_file, dtype=np.float16).reshape([2, 1024, 1024, 6])
    result = open('./result.txt', mode = 'w')
    print(sum(c - op_output))
    if(np.count_nonzero(sum(c - op_output) == 0)): 
        result.write('Project Run Success \n')


if __name__ == '__main__':
    # print(sys.argv)
    if(len(sys.argv) < 4):
        print("paras is not ok")
        sys.exit()
    
    compute(sys.argv[1], sys.argv[2], sys.argv[3])
