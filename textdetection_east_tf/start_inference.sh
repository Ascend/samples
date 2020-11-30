#! /bin/bash
input="input/"
output="output/"
model="model/east_text_detection.om"
image_input="image_input/"
image_output="image_output/"

rm -rf $input/
rm -rf $output/
rm -rf $image_output
mkdir $output
mkdir $input
mkdir $image_output
#preprocess
python3 preprocess.py $image_input $input

#start infence
./msame --model $model --input $input --output $output

#postprocess
python3 postprocess.py $image_input $output $image_output
