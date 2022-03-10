# create test data

input_shape='[2,1024,1024,3]'  # NHWC
filter_shape='[6,3,3,3]'       # NCHW 
output_shape='[2,1024,1024,6]' # NHWC

input_file=$(python3 tools_generate_data.py x -s ${input_shape} -r [1,10] -d float16)
filter_file=$(python3 tools_generate_data.py filter -s ${filter_shape} -r [1,3] -d float16)
output_file='out_float16_2x1024x1024x6.bin.in'

mv ${input_file} ../data
mv ${filter_file} ../data

../out/main '../data/'$input_file '../data/'$filter_file '../data/'$output_file


if tensorflow=$(python3 -c "import tensorflow;print(tensorflow.__version__)" 2>/dev/null);then
    if [ ${tensorflow} != 1.15.0 ];then
        pip3 install tensorflow==1.15.0 --user 2>/dev/null
    fi
else
    pip3 install tensorflow==1.15.0 --user 2>/dev/null
fi



python3 computebytf.py '../data/'$input_file '../data/'$filter_file '../data/'$output_file
