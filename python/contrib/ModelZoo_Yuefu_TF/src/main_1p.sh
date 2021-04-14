#!/bin/bash
dir="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
echo ${dir}
export JOB_ID=10086
export DEVICE_ID=1
export RANK_ID=0
export RANK_SIZE=1
export RANK_TABLE_FILE=${dir}/../src/device_table_1p.json
export USE_NPU=True
export POETRY_TYPE=-1
export max_decode_len=80
python3.7 ${dir}/../src/poetry_v2.py --title="中秋" --type="七言绝句"
