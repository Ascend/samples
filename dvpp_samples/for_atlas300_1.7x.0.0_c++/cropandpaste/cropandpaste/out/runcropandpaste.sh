cd ../build/intermediates/host
cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
make
cd ../../../out
./cropandpaste ../data/wood_rabbit_1024_1068_nv12.yuv 1024 1068 output.yuv 224 224
python3 yuv2jpg.py

