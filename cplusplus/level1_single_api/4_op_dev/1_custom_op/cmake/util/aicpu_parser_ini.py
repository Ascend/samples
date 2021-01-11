# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
import stat
import sys

def parse_ini_files(ini_files):
    aicpu_ops_info = {}
    for ini_file in ini_files:
        parse_ini_to_obj(ini_file, aicpu_ops_info)
    return aicpu_ops_info

def parse_ini_to_obj(ini_file, aicpu_ops_info):
    with open(ini_file) as ini_file:
        lines = ini_file.readlines()
        op = {}
        for line in lines:
            line = line.rstrip()
            if line.startswith("["):
                op_name = line[1:-1]
                op = {}
                aicpu_ops_info[op_name] = op
            else:
                key1 = line[:line.index("=")]
                key2 = line[line.index("=")+1:]
                key1_0, key1_1 = key1.split(".")
                if not key1_0 in op:
                    op[key1_0] = {}
                op[key1_0][key1_1] = key2

def check_custom_op_opInfo(required_custom_op_info_keys, key, op, op_key, custom_ops):
    op_info = op["opInfo"]
    missing_keys=[]
    for required_op_info_key in required_custom_op_info_keys:
        if not required_op_info_key in op_info:
                    missing_keys.append(required_op_info_key)
    if len(missing_keys) > 0:
        print("op: " + op_key + " opInfo missing: " + ",".join(missing_keys))
        raise KeyError("bad key value")

def check_op_opInfo(required_op_info_keys, required_custom_op_info_keys, key, op, op_key, custom_ops):
    op_info = op["opInfo"]
    missing_keys=[]
    for required_op_info_key in required_op_info_keys:
        if not required_op_info_key in op_info:
                    missing_keys.append(required_op_info_key)
    if len(missing_keys) > 0:
        print("op: " + op_key + " opInfo missing: " + ",".join(missing_keys))
        raise KeyError("bad key value")
    elif op_info["opKernelLib"] == "CUSTAICPUKernel":
        check_custom_op_opInfo(required_custom_op_info_keys, key, op, op_key, custom_ops)
        op["opInfo"]["userDefined"] = "True"

def check_op_input_output(info, key, op):
    for op_sets in op[key]:
        if (op_sets not in ('format', 'type', 'name')):
            print(info + " should has format type or name as the key, "
                    + "but getting " + op_sets)
            raise KeyError("bad op_sets key")

def check_op_info(aicpu_ops):
    print("==============check valid for aicpu ops info start==============")
    required_op_info_keys = ["computeCost", "engine", "flagAsync", "flagPartial", "opKernelLib"]
    required_custom_op_info_keys = ["kernelSo", "functionName", "workspaceSize"]
    custom_ops = []
    for op_key in aicpu_ops:
        op = aicpu_ops[op_key]
        for key in op:
            if key == "opInfo":
                check_op_opInfo(required_op_info_keys, required_custom_op_info_keys, key, op, op_key, custom_ops)

            elif (key[:5] == "input") and (key[5:].isdigit()):
                check_op_input_output("input", key, op)
            elif (key[:6] == "output") and (key[6:].isdigit()):
                check_op_input_output("output", key, op)
            elif (key[:13] == "dynamic_input") and (key[13:].isdigit()):
                check_op_input_output("dynamic_input", key, op)
            elif (key[:14] == "dynamic_output") and (key[14:].isdigit()):
                check_op_input_output("dynamic_output", key, op)
            else:
                print("Only opInfo, input[0-9], output[0-9] can be used as a key, "
                      "but op %s has the key %s" %(op_key, key))
                raise KeyError("bad key value")
    print("==============check valid for aicpu ops info end================\n")

def write_json_file(aicpu_ops_info, json_file_path):
    json_file_real_path = os.path.realpath(json_file_path)
    with open(json_file_real_path, "w") as f:
        # Only the owner and group have rights
        os.chmod(json_file_real_path, stat.S_IWGRP + stat.S_IWUSR + stat.S_IRGRP + stat.S_IRUSR)
        json.dump(aicpu_ops_info, f, sort_keys=True, indent=4, separators=(',', ':'))
    print("Compile aicpu op info cfg successfully.")

def parse_ini_to_json(ini_file_paths, outfile_path):
    aicpu_ops_info = parse_ini_files(ini_file_paths)
    try:
        check_op_info(aicpu_ops_info)
        write_json_file(aicpu_ops_info, outfile_path)
    except KeyError:
        print("bad format key value, failed to generate json file")

if __name__ == '__main__':
    args = sys.argv

    outfile_path = "tf_kernel.json"
    ini_file_paths = []

    for arg in args:
        if arg.endswith("ini"):
            ini_file_paths.append(arg)
        if arg.endswith("json"):
            outfile_path = arg

    if len(ini_file_paths) == 0:
        ini_file_paths.append("tf_kernel.ini")

    parse_ini_to_json(ini_file_paths, outfile_path)
