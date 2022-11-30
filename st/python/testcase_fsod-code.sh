script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../best_practices/contrib/fsod-code
project_path=${script_path}

function setEnv() {
  source /home/HwHiAiUser/.bashrc
  source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
}

function main() {

    # 下载数据集
    cd ${project_path}
    mkdir -p ${project_path}/datasets
    if [ ! -f "${project_path}/datasets/VOC-custom.zip" ];then
        wget -O ${project_path}/datasets/VOC-custom.zip https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/fsod-code/VOC-custom.zip --no-check-certificate
        #unzip -n -d ${project_path}/datasets ${project_path}/datasets/VOC-custom.zip
    fi
    setEnv
    unzip -n -d ${project_path}/datasets ${project_path}/datasets/VOC-custom.zip
    # 转模型
    if [ ! -f "${project_path}/fsdet_model.om" ];then
        wget -O ${project_path}/model_py1.8.onnx https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/fsod-code/model_py1.8.onnx --no-check-certificate
        atc --framework=5 --model=model_py1.8.onnx --output=fsdet_model --input_format=NCHW --input_shape="0:1,3,800,800" --out_nodes="Gather_1692:0;Cast_1689:0;Reshape_1683:0" --soc_version=Ascend310
    fi

    # 运行
    python3.6 tools/deploy/fsdet_deploy.py --model=fsdet_model.om \
            --height=800 --width=800 --mode=metric --txt=datasets/VOC-custom/ImageSets/Main/test.txt \
            --split=test --data_dir=datasets/VOC-custom/ --cf_thres=0.05 --iou_thres=0.5
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
