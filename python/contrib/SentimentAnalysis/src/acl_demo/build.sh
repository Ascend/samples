build_dir=./build/
if [  ! -d "$build_dir" ];then
    mkdir -p $build_dir
fi

cd build/
cmake ../ -DCMAKE_CXX_COMPILER=g++
make
# cp inference ../run/out
