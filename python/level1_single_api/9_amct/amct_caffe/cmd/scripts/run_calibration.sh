set -e

model_name=ResNet-50-model
model=./model/ResNet-50-deploy.prototxt
weight=./model/ResNet-50-model.caffemodel
input_shape="data:1,3,224,224"
data_dir="data/image"

python3.7 ./src/process_data.py
amct_caffe calibration --model $model  --weights $weight --save_path ./results/resnet50_ --input_shape $input_shape --data_type "float32" --data_dir $data_dir