presenter_server_name="display"
project_name="cplusplus_ascendcamera"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..
common_script_dir=${script_path}/../../../../../common/
run_command="./main -i -c 0 -o ./output/filename.jpg --overwrite"

. ${common_script_dir}/testcase_common.sh

function main() {

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    cd ${project_path}/out
    mkdir ${project_path}/out/output
    ${run_command}
    if [ ! -f "${project_path}/out/output/filename.jpg" ];then
        return ${ret}
    fi

    return ${success}
}
main
