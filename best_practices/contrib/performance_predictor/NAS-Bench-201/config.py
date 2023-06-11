import tensorflow as tf

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('train_dir', 'dataset/', """Directory where to write event logs """"""and checkpoint.""")
tf.app.flags.DEFINE_integer('max_steps', 39000, """Number of batches to run.""")
tf.app.flags.DEFINE_boolean('log_device_placement', False, """Whether to log device placement.""")
tf.app.flags.DEFINE_integer('batch_size', 256,  """Number of images to process in a batch.""")
tf.app.flags.DEFINE_integer('card_id', 5,  """Logic ID of Ascend card""")
tf.app.flags.DEFINE_string('data_dir', 'dataset/', """Path to the CIFAR-10 data directory.""")
tf.app.flags.DEFINE_float('initial_lr', 0.1, 'Initial learning rate.')
tf.app.flags.DEFINE_float('end_lr', 0.0, 'End/Min learning rate.')
tf.app.flags.DEFINE_float('momentum', 0.9, 'momentum')
tf.app.flags.DEFINE_list(
    'train_data_files', ['dataset/train_1.tfrecords', 'dataset/train_2.tfrecords', 'dataset/train_3.tfrecords', 'dataset/train_4.tfrecords'],
    'Training data files in TFRecord format. Multiple files can be passed in a'
    ' comma-separated list. The first file in the list will be used for'
    ' computing the training error.')
tf.app.flags.DEFINE_string('valid_data_file', 'dataset/validation.tfrecords', 'Validation data in TFRecord format.')
tf.app.flags.DEFINE_string('test_data_file', 'dataset/test.tfrecords', 'Testing data in TFRecord format.')
tf.app.flags.DEFINE_string('sample_data_file', 'dataset/sample.tfrecords', 'Sampled batch data in TFRecord format.')
tf.app.flags.DEFINE_list('training_arch_list', [0, 1], 'training arch file id')
tf.app.flags.DEFINE_string('arch_file_path', 'training_arch/', 'arch file path')
tf.app.flags.DEFINE_string('result_file_path', 'result/', 'result file path')
