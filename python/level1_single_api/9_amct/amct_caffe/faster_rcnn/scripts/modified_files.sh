#!/bin/bash
set -e
RUN_MODE=$1
CAFFE_DIR=$2
PYTHON3_V=${3}
PYTHON3_INCLUDE_DIR=$4
WITH_BENCHMARK=$5

SAMPLE_SRC_DIR=../src/

echo "[INFO]Run modified file in ${RUN_MODE} mode"
######modified ${SAMPLE_SRC_DIR}/python_tools/setup.py#######
# uncomment noneed part
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --comment --line --section 142 148
# Modified complie of cypthon setup from python2 to python3
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --replace --content --all "include_dirs = [numpy_include]" --dest "include_dirs = [numpy_include, '${PYTHON3_INCLUDE_DIR}']"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --replace --content --all "include_dirs = [numpy_include, CUDA['include']]" --dest "include_dirs = [numpy_include, CUDA['include'], '${PYTHON3_INCLUDE_DIR}']"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --replace --content --all "include_dirs = [numpy_include, 'pycocotools']" --dest "include_dirs = [numpy_include, 'pycocotools', '${PYTHON3_INCLUDE_DIR}']]"

if [ $RUN_MODE == CPU ]
then
    # comment part of compiler for gpu
    python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --comment --line --section 125 141
    python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/setup.py --comment --line --single 58
fi

# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/setup.py

######modified ${SAMPLE_SRC_DIR}/python_tools/rpn/proposal_layer.py#######
# Fix syntax error
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/rpn/proposal_layer.py --replace --content --first "param_str_" --dest "param_str"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/rpn/proposal_layer.py --replace --content --first "str(self.phase) # either 'TRAIN' or 'TEST'" --dest "'TEST'"
# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/rpn/proposal_layer.py

# ######modified ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/nms_wrapper.py#######
# comment part of compiler for gpu
if [ $RUN_MODE == CPU ]
then
    python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/nms_wrapper.py --comment --line --single 9
fi
# #######modified ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/test.py######
# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/test.py
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/test.py --replace --line --single 295 --dest "    return imdb.evaluate_detections(all_boxes, output_dir)"

# #######modified ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/config.py######
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/fast_rcnn/config.py

# #######modified ${SAMPLE_SRC_DIR}/python_tools/rpn/generate_anchors.py######
# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/rpn/generate_anchors.py

# #######modified ${SAMPLE_SRC_DIR}/python_tools/utils/blob.py######
# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/utils/blob.py

# #######modified ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py######
# trans syntax python2 to python3
2to3 -w ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py
# comment unused module
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --comment --line --single 16
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --replace --content --first "cfg.DATA_DIR, 'demo', image_name" --dest "cfg.DATA_DIR, image_name"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --replace --line --single 40 --dest "def vis_detections(im, class_name, dets, image_name, thresh=0.5, is_quantize=False):"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 71 --dest "    if not is_quantize:"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 72 --dest "        file_dir = 'pre_detect_results'"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 73 --dest "        if not os.path.isdir(file_dir):"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 74 --dest "            os.makedirs(file_dir)"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 75 --dest "    else:"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 76 --dest "        file_dir = 'quant_detect_results'"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 77 --dest "        if not os.path.isdir(file_dir):"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 78 --dest "            os.makedirs(file_dir)"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 79 --dest "    plt.savefig('./{}/{}_{}'.format(file_dir, class_name, image_name))"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --replace --line --single 81 --dest "def demo(net, image_name, is_quantize):"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --replace --line --single 107 --dest "        vis_detections(im, cls, dets, image_name, thresh=CONF_THRESH, is_quantize=is_quantize)"

python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 20 --dest "import matplotlib"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 21 --dest "gui_env = ['TKAgg','Agg']"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 22 --dest "for gui in gui_env:"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 23 --dest "    try:"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 24 --dest "        print('testing:', gui)"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 25 --dest "        matplotlib.use(gui,warn=False, force=True)"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 26 --dest "        break"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 27 --dest "    except:"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 28 --dest "        continue"
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/tools/demo.py --insert --line --single 29 --dest "print('Using:',matplotlib.get_backend())"

# Set caffe_dir
# #######modified ${SAMPLE_SRC_DIR}/init_paths_for_amct.py######
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/init_paths_for_amct.py --replace --content --first "your_caffe_master_dir" --dest "${CAFFE_DIR}"

# #######modified ${SAMPLE_SRC_DIR}/python_tools/Makefile######
python3 file_operator.py --file ${SAMPLE_SRC_DIR}/python_tools/Makefile --replace --content --first "python" --dest "${PYTHON3_V}"

if [[ $WITH_BENCHMARK == 'with_benchmark' ]]; then
    #statements
    # modified ${SAMPLE_SRC_DIR}/python_tools/tools/test_net.py
    TARGET_FILE=${SAMPLE_SRC_DIR}/python_tools/tools/test_net.py
    2to3 -w ${TARGET_FILE}
    python3 file_operator.py --file ${TARGET_FILE} --replace --line --single 90 --dest "    return test_net(net, imdb, max_per_image=args.max_per_image, vis=args.vis)"
    python3 file_operator.py --file ${TARGET_FILE} --comment --line --list 12 60 61
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 61 --dest "def test_voc_2007(args):"

    python3 file_operator.py --file ${TARGET_FILE} --comment --line --list 72 81 82
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 83 --dest "    if args.gpu_id is None:"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 84 --dest "        cfg.USE_GPU_NMS = False"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 85 --dest "        caffe.set_mode_cpu()"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 86 --dest "    else:"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 87 --dest "        caffe.set_mode_gpu()"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 88 --dest "        caffe.set_device(args.gpu_id)"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 89 --dest "        cfg.GPU_ID = args.gpu_id"

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/coco.py
    2to3 -w ${SAMPLE_SRC_DIR}/python_tools/datasets/coco.py

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/ds_utils.py
    2to3 -w ${SAMPLE_SRC_DIR}/python_tools/datasets/ds_utils.py

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/factory.py
    2to3 -w ${SAMPLE_SRC_DIR}/python_tools/datasets/factory.py

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/imdb.py
    2to3 -w ${SAMPLE_SRC_DIR}/python_tools/datasets/imdb.py

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/pascal_voc.py
    TARGET_FILE=${SAMPLE_SRC_DIR}/python_tools/datasets/pascal_voc.py
    2to3 -w ${TARGET_FILE}
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 304 --dest "        return np.mean(aps)"
    python3 file_operator.py --file ${TARGET_FILE} --replace --line --single 323 --dest "        mAP = self._do_python_eval(output_dir)"
    python3 file_operator.py --file ${TARGET_FILE} --insert --line --single 332 --dest "        return mAP"
    python3 file_operator.py --file ${TARGET_FILE} --replace --line --single 288 --dest "            with open(os.path.join(output_dir, cls + '_pr.pkl'), 'wb') as f:"

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/voc_eval.py
    TARGET_FILE=${SAMPLE_SRC_DIR}/python_tools/datasets/voc_eval.py
    2to3 -w ${TARGET_FILE}
    python3 file_operator.py --file ${TARGET_FILE} --replace  --line --single 115 --dest "        with open(cachefile, 'wb') as f:"
    python3 file_operator.py --file ${TARGET_FILE} --replace  --line --single 119 --dest "        with open(cachefile, 'rb') as f:"

    # modified ${SAMPLE_SRC_DIR}/python_tools/datasets/tools/mcg_munge.py
    2to3 -w ${SAMPLE_SRC_DIR}/python_tools/datasets/tools/mcg_munge.py
fi

TARGET_FILE=${SAMPLE_SRC_DIR}/faster_rcnn_sample.py
if [[ $WITH_BENCHMARK == 'with_benchmark' ]]; then
    python3 file_operator.py --file ${TARGET_FILE} --comment --line --list 40 41 42 72 73 88 89 90 138 139 140 141
    python3 file_operator.py --file ${TARGET_FILE} --uncomment --line --list 91 92 142 143
else
    python3 file_operator.py --file ${TARGET_FILE} --uncomment --line --list 40 41 42 72 73 88 89 90 138 139 140 141
    python3 file_operator.py --file ${TARGET_FILE} --comment --line --list 91 92 142 143
fi
