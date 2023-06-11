#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
echo $ScriptPath
if [ ! -d ${ScriptPath}/../out ];then
mkdir ${ScriptPath}/../out
else
echo "[INFO] The out directory is already there"
fi
cd ${ScriptPath}/../src
echo "[INFO] The sample starts to run"
python3.7 sampleYOLOV7NMSONNX.py
if [ $? -ne 0 ];then
echo "[INFO] The program runs failed"
else
echo "[INFO] The program runs successfully"
fi