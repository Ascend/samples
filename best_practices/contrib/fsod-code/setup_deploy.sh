#!/bin/bash

pip3 install onnx==1.8.0
pip3 install -U numpy
pip3 install opencv_python
pip3 install tqdm
pip3 install tabulate

# Install Apex
git clone https://github.com/ptrblck/apex.git
cd apex
git checkout apex_no_distributed
pip3 install -v --no-cache-dir ./
cd ..

# Install Detectron2
git clone https://github.com/facebookresearch/detectron2 detectron2_deploy
cd detectron2_deploy
git checkout v0.5
python3 -m pip install -e ./

#git apply ../fsdet_deploy.diff

