# Copyright 2020-2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
parser ini to json
"""

import json
import os
import stat
import sys


def parse_ini_files(ini_files):
    """
    parse ini files to json
    Parameters:
    ----------------
    ini_files:input file list
    return:ops_info
    ----------------
    """
    tbe_ops_info = {}
    for ini_file in ini_files:
        parse_ini_to_obj(ini_file, tbe_ops_info)
    return tbe_ops_info


def parse_ini_to_obj(ini_file, tbe_ops_info):
    """
    parse ini file to json obj
    Parameters:
    ----------------
    ini_file:ini file path
    tbe_ops_info:ops_info
    ----------------
    """
    with open(ini_file) as ini_file:
        lines = ini_file.readlines()
        op_dict = {}
        op_name = ""
        for line in lines:
            line = line.rstrip()
            if line == "":
                continue
            if line.startswith("["):
                op_name = line[1:-1]
                op_dict = {}
                tbe_ops_info[op_name] = op_dict
            else:
                key1 = line[:line.index("=")]
                key2 = line[line.index("=")+1:]
                key1_0, key1_1 = key1.split(".")
                if not key1_0 in op_dict:
                    op_dict[key1_0] = {}
                if key1_1 in op_dict[key1_0]:
                    raise RuntimeError("Op:" + op_name + " " + key1_0 + " " +
                                       key1_1 + " is repeated!")
                op_dict[key1_0][key1_1] = key2


def check_op_info(tbe_ops):
    """
    Function Description:
        Check info.
    Parameter: tbe_ops
    Return Value: is_valid
    """
    print("\n\n==============check valid for ops info start==============")
    required_op_input_info_keys = ["paramType", "name"]
    required_op_output_info_keys = ["paramType", "name"]
    param_type_valid_value = ["dynamic", "optional", "required"]
    is_valid = True
    for op_key in tbe_ops:
        op_dict = tbe_ops[op_key]

        for op_info_key in op_dict:
            if op_info_key.startswith("input"):
                op_input_info = op_dict[op_info_key]
                missing_keys = []
                for required_op_input_info_key in required_op_input_info_keys:
                    if not required_op_input_info_key in op_input_info:
                        missing_keys.append(required_op_input_info_key)
                if len(missing_keys) > 0:
                    print("op: " + op_key + " " + op_info_key + " missing: " +
                          ",".join(missing_keys))
                    is_valid = False
                else:
                    if not op_input_info["paramType"] in param_type_valid_value:
                        print("op: " + op_key + " " + op_info_key + \
                              " paramType not valid, valid key:[dynamic, "
                              "optional, required]")
                        is_valid = False
            if op_info_key.startswith("output"):
                op_input_info = op_dict[op_info_key]
                missing_keys=[]
                for required_op_input_info_key in required_op_output_info_keys:
                    if not required_op_input_info_key in op_input_info:
                        missing_keys.append(required_op_input_info_key)
                if len(missing_keys) > 0:
                    print("op: " + op_key + " " + op_info_key + " missing: " +
                          ",".join(missing_keys))
                    is_valid = False
                else:
                    if not op_input_info["paramType"] in param_type_valid_value:
                        print("op: " + op_key + " " + op_info_key +
                              " paramType not valid, valid key:[fix, range, "
                              "list]")
                        is_valid = False
    print("==============check valid for ops info end================\n\n")
    return is_valid


def write_json_file(tbe_ops_info, json_file_path):
    """
    Save info to json file
    Parameters:
    ----------------
    tbe_ops_info: ops_info
    json_file_path: json file path
    ----------------
    """
    json_file_real_path = os.path.realpath(json_file_path)
    with open(json_file_real_path, "w") as file_path:
        # Only the owner and group have rights
        os.chmod(json_file_real_path, stat.S_IWGRP + stat.S_IWUSR + stat.S_IRGRP
                 + stat.S_IRUSR)
        json.dump(tbe_ops_info, file_path, sort_keys=True, indent=4,
                  separators=(',', ':'))
    print("Compile op info cfg successfully.")


def parse_ini_to_json(ini_file_paths, outfile_path):
    """
    parse ini files to json file
    Parameters:
    ----------------
    ini_file_paths: list of ini file path
    outfile_path: output file path
    ----------------
    """
    tbe_ops_info = parse_ini_files(ini_file_paths)
    if not check_op_info(tbe_ops_info):
        print("Compile op info cfg failed.")
        return False
    write_json_file(tbe_ops_info, outfile_path)
    return True


if __name__ == '__main__':
    args = sys.argv

    output_file_path = "tbe_ops_info.json"
    ini_file_path_list = []

    for arg in args:
        if arg.endswith("ini"):
            ini_file_path_list.append(arg)
            output_file_path = arg.replace(".ini", ".json")
        if arg.endswith("json"):
            output_file_path = arg

    if len(ini_file_path_list) == 0:
        ini_file_path_list.append("tbe_ops_info.ini")

    if not parse_ini_to_json(ini_file_path_list, output_file_path):
        sys.exit(1)
    sys.exit(0)
