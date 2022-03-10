script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../cplusplus/level2_simple_inference/6_other/acl_compile_and_execute_conv2d/scripts
common_script_dir=${script_path}
Kernel=`uname -m`
.  ${common_script_dir}/../../../../../common/testcase_common.sh
function main() {
    setEnv
    mkdir -p ${script_path}/../data 
    mkdir -p ${script_path}/../build/intermediates/host
    if [[ ${Kernel} = "x86_64" ]];then
        TargetKernel="x86"
        cxx_compiler="g++"
    else
        TargetKernel="arm"
        cxx_compiler="aarch64-linux-gnu-g++"
    fi
    cd ${script_path}/../build/intermediates/host && cmake ${script_path}/../src -DCMAKE_CXX_COMPILER=${cxx_compiler} -DCMAKE_SKIP_RPATH=TRUE && make
    cd ${script_path} && > result.txt && bash sample_run.sh

    if [ -s ${script_path}/result.txt ];then
        return ${success}
    else
        return ${verifyResError}
    fi
}
main
