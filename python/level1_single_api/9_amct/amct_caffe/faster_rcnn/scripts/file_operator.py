#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

file operator

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import argparse


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='File Operator')

    parser.add_argument('--file', dest='file_name',
                        help='Specify the file to operate',
                        default=None, type=str)

    parser.add_argument('--replace', dest='operator_replace',
                        help='Do replace mode.',
                        action='store_true')
    parser.add_argument('--insert', dest='operator_insert',
                        help='Do insert mode.',
                        action='store_true')
    parser.add_argument('--delete', dest='operator_delete',
                        help='Do delete mode.',
                        action='store_true')
    parser.add_argument('--comment', dest='operator_comment',
                        help='Do comment mode.',
                        action='store_true')
    parser.add_argument('--uncomment', dest='operator_uncomment',
                        help='Do uncomment mode.',
                        action='store_true')

    parser.add_argument('--lines', dest='point_lines',
                        help='Specify operator position by line',
                        action='store_true')
    parser.add_argument('--single', dest='mode_single', nargs='+',
                        help='Replace single line specified by line num',
                        default=None, type=int)
    parser.add_argument('--list', dest='mode_list', nargs='+',
                        help='Replace list line specified by line num',
                        default=None, type=int)
    parser.add_argument('--section', dest='mode_section', nargs='+',
                        help='Replace section line specified by line num',
                        default=None, type=int)

    parser.add_argument('--content', dest='point_content',
                        help='Specify operator position by content',
                        action='store_true')
    parser.add_argument('--all', dest='mode_all',
                        help='Replace each place specified by content',
                        default=None, type=str)
    parser.add_argument('--first', dest='mode_first',
                        help='Replace the first one specified by content',
                        default=None, type=str)
    parser.add_argument('--last', dest='mode_last',
                        help='Replace the last one specified by content',
                        default=None, type=str)

    parser.add_argument('--dest', dest='point_dest',
                        help='Specify the replace dest',
                        default=None, type=str)
    return parser.parse_args()

OPERATOR_ARGS = parse_args()


def check_operator(args, file_operator):
    """
    Function: Check validity of operator in args
    Parameters: args: args to check
                file_operator: file operator parsed from args
    Return: None
    """
    def check_operator_replace_delete(args, file_operator):
        """check opeartor of replac and delete"""
        if args.operator_replace:
            file_operator['operator'] = 'replace'
        if args.operator_delete:
            if file_operator['operator'] is None:
                file_operator['operator'] = 'delete'
            else:
                raise ValueError('Only support one operator one time')
    check_operator_replace_delete(args, file_operator)

    def check_operator_insert(args, file_operator):
        """check opeartor of insert"""
        if args.operator_insert:
            if file_operator['operator'] is None:
                file_operator['operator'] = 'insert'
            else:
                raise ValueError('Only support one operator one time')
    check_operator_insert(args, file_operator)

    def check_operator_coment_uncoment(args, file_operator):
        """check opeartor of coment and uncoment"""
        if args.operator_comment:
            if file_operator['operator'] is None:
                file_operator['operator'] = 'comment'
            else:
                raise ValueError('Only support one operator one time')
        if args.operator_uncomment:
            if file_operator['operator'] is None:
                file_operator['operator'] = 'uncomment'
            else:
                raise ValueError('Only support one operator one time')
    check_operator_coment_uncoment(args, file_operator)


def check_point(args, file_operator):
    """
    Function: Check validity of point in args
    Parameters: args: args to check
                file_operator: file operator parsed from args
    Return: None
    """
    if int(args.point_lines) + int(args.point_content) != 1:
        raise ValueError('Must and only can specify one point one time')
    file_operator['point'] = 'lines' if args.point_lines else 'content'

    if int(args.point_dest is not None) ^ int(file_operator['operator'] in ['replace', 'insert']):
        raise ValueError('Only and must specify dest in operator replace or insert.')

    file_operator['dest'] = args.point_dest


def check_mode_insert(args, file_operator):
    """
    Function: Check insert mode in args
    Parameters: args: args to check
                file_operator: file operator parsed from args
    Return: None
    """

    if args.mode_single is not None:
        def check_mode_insert_single(args, file_operator):
            """check_mode_insert_single"""
            if file_operator['point'] != 'lines':
                raise ValueError('Only support mode single in point lines')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'single'
            file_operator['mode_value'] = args.mode_single
        check_mode_insert_single(args, file_operator)
    if args.mode_list is not None:
        def check_mode_insert_lines(args, file_operator):
            """check_mode_insert_lines"""
            if file_operator['point'] != 'lines':
                raise ValueError('Only support mode list in point lines')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'list'
            file_operator['mode_value'] = args.mode_list
        check_mode_insert_lines(args, file_operator)
    if args.mode_section is not None:
        def check_mode_insert_section(args, file_operator):
            """check_mode_insert_section"""
            if file_operator['point'] != 'lines':
                raise ValueError('Only support mode section in point lines')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'section'
            if len(args.mode_section) != 2:
                raise ValueError('Mode section must specified two line number')
            if args.mode_section[1] < args.mode_section[0]:
                raise ValueError('Mode section specified line number must from begin to end')
            file_operator['mode_value'] = args.mode_section
        check_mode_insert_section(args, file_operator)


def check_mode_replace(args, file_operator):
    """
    Function: Check replace mode in args
    Parameters: args: args to check
                file_operator: file operator parsed from args
    Return: None
    """
    if args.mode_first is not None:
        def check_replace_first(args, file_operator):
            """check_replace_first"""
            if file_operator['point'] != 'content':
                raise ValueError('Only support mode first in point content')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'first'
            file_operator['mode_value'] = args.mode_first
        check_replace_first(args, file_operator)
    if args.mode_last is not None:
        def check_replace_last(args, file_operator):
            """check_replace_last"""
            if file_operator['point'] != 'content':
                raise ValueError('Only support mode last in point content')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'last'
            file_operator['mode_value'] = args.mode_last
        check_replace_last(args, file_operator)
    if args.mode_all is not None:
        def check_replace_all(args, file_operator):
            """check_replace_all"""
            if file_operator['point'] != 'content':
                raise ValueError('Only support mode all in point content')
            if file_operator['mode'] is not None:
                raise ValueError('Only support one mode one time')
            file_operator['mode'] = 'all'
            file_operator['mode_value'] = args.mode_all
        check_replace_all(args, file_operator)


def args_check(args):
    """check args"""
    file_operator = {
        'file': None,
        'operator': None,
        'point': None,
        'mode': None,
        'mode_value': None,
        'dest': None
        }
    # check file
    if args.file_name is None:
        raise ValueError('Must specify a file to operate')
    file_operator['file'] = os.path.realpath(args.file_name)
    # check operator
    check_operator(args, file_operator)
    # check point
    check_point(args, file_operator)
    # check mode
    check_mode_insert(args, file_operator)
    check_mode_replace(args, file_operator)

    return file_operator


def replace_operator(file_operator):
    """replace operation"""
    if file_operator['point'] == 'lines':

        def replace_lines_mode(file_operator):
            """replace_lines_mode"""
            file_operator['dest'] = '{}\n'.format(OPERATOR_ARGS.point_dest)
            file_lines = None
            with open(file_operator['file'], 'r') as file_open:
                file_lines = file_open.readlines()
            for line in file_operator['mode_value']:
                if line > len(file_lines):
                    raise ValueError('Specified line {} is out of range:{}'.format(line, len(file_lines)))
            if file_operator['mode'] == 'single':
                file_lines[file_operator['mode_value'][0] - 1] = file_operator['dest']
            if file_operator['mode'] == 'list':
                for line in file_operator['mode_value']:
                    file_lines[line - 1] = file_operator['dest']
            if file_operator['mode'] == 'section':
                for line in range(file_operator['mode_value'][0] - 1, file_operator['mode_value'][1]):
                    file_lines[line] = file_operator['dest']
            with open(file_operator['file'], 'w') as file_open:
                file_open.writelines(file_lines)
        replace_lines_mode(file_operator)
    elif file_operator['point'] == 'content':
        file_contents = None
        with open(file_operator['file'], 'r') as file_open:
            file_contents = str(file_open.read())
        if file_operator['mode'] == 'first':
            file_contents = file_contents.replace(file_operator['mode_value'], file_operator['dest'], 1)
        if file_operator['mode'] == 'all':
            file_contents = file_contents.replace(file_operator['mode_value'], file_operator['dest'])
        with open(file_operator['file'], 'w') as file_open:
            file_open.write(file_contents)


def delete_operator(file_operator):
    """delete operation"""
    if file_operator['point'] == 'lines':
        file_lines = None
        with open(file_operator['file'], 'r') as file_open:
            file_lines = file_open.readlines()
        for line in file_operator['mode_value']:
            if line >= len(file_lines):
                raise ValueError('Specified line {} is out of range'.format(line))

        def do_delete_in_lines_mode(file_operator, file_lines):
            """do_delete_in_lines_mode"""
            if file_operator['mode'] == 'single':
                del file_lines[file_operator['mode_value'][0] - 1]
            if file_operator['mode'] == 'list':
                for line in file_operator['mode_value'][::-1]:
                    print('line:', line)
                    del file_lines[line - 1]
            if file_operator['mode'] == 'section':
                for line in range(file_operator['mode_value'][0] - 1, file_operator['mode_value'][1])[::-1]:
                    del file_lines[line]
        do_delete_in_lines_mode(file_operator, file_lines)
        with open(file_operator['file'], 'w') as file_open:
            file_open.writelines(file_lines)
    elif file_operator['point'] == 'content':
        file_contents = None
        with open(file_operator['file'], 'r') as file_open:
            file_contents = str(file_open.read())

        def do_delete_in_content_mode(file_operator, file_contents):
            """do_delete_in_content_mode"""
            if file_operator['mode'] == 'first':
                file_contents = file_contents.replace(file_operator['mode_value'], '', 1)
            if file_operator['mode'] == 'all':
                file_contents = file_contents.replace(file_operator['mode_value'], '')
        do_delete_in_content_mode(file_operator, file_contents)
        with open(file_operator['file'], 'w') as file_open:
            file_open.write(file_contents)


def insert_operator(file_operator):
    """insert operation"""
    if file_operator['point'] == 'lines':
        file_operator['dest'] = '{}\n'.format(OPERATOR_ARGS.point_dest)
        file_lines = None
        with open(file_operator['file'], 'r') as file_open:
            file_lines = file_open.readlines()
        for line in file_operator['mode_value']:
            if line >= len(file_lines):
                raise ValueError('Specified line {} is out of range'.format(line))

        def do_insert_in_lines_mode(file_operator, file_lines):
            """do_insert_in_lines_mode"""
            if file_operator['mode'] == 'single':
                file_lines.insert(file_operator['mode_value'][0] - 1, file_operator['dest'])
            if file_operator['mode'] == 'list':
                expand_num = 0
                for line in file_operator['mode_value']:
                    file_lines.insert(line - 1 + expand_num, file_operator['dest'])
                    expand_num += 1
            if file_operator['mode'] == 'section':
                expand_num = 0
                for line in range(file_operator['mode_value'][0] - 1, file_operator['mode_value'][1]):
                    file_lines.insert(line + expand_num, file_operator['dest'])
                    expand_num += 1
        do_insert_in_lines_mode(file_operator, file_lines)

        with open(file_operator['file'], 'w') as file_open:
            file_open.writelines(file_lines)
    elif file_operator['point'] == 'content':
        file_contents = None
        with open(file_operator['file'], 'r') as file_open:
            file_contents = str(file_open.read())

        def do_insert_in_content_mode(file_operator, file_contents):
            """do_insert_in_content_mode"""
            if file_operator['mode'] == 'first':
                file_contents = file_contents.replace(file_operator['mode_value'], \
                    '{}{}'.format(file_operator['mode_value'], file_operator['dest']), 1)
            if file_operator['mode'] == 'all':
                file_contents = file_contents.replace(file_operator['mode_value'], \
                    '{}{}'.format(file_operator['mode_value'], file_operator['dest']))
        do_insert_in_content_mode(file_operator, file_contents)
        with open(file_operator['file'], 'w') as file_open:
            file_open.write(file_contents)


def comment_operator(file_operator):
    """comment operation"""
    if file_operator['point'] == 'lines':
        file_lines = None
        with open(file_operator['file'], 'r') as file_open:
            file_lines = file_open.readlines()
        for line in file_operator['mode_value']:
            if line >= len(file_lines):
                raise ValueError('Specified line {} is out of range'.format(line))

        def comment_operation(string):
            """comment_operation"""
            striped_str = string.strip()
            if len(striped_str) and striped_str.startswith('#'):
                return string
            for index, char in enumerate(string):
                if char != ' ' and index < len(string) - 1 and char != '#':
                    return '{}{}{}'.format(string[0:index], '# ',
                                           string[index:len(string)])
            return string

        def do_comment_in_lines_mode(file_operator, file_lines):
            """do_comment_in_lines_mode"""
            if file_operator['mode'] == 'single':
                file_lines[file_operator['mode_value'][0] - 1] = comment_operation( \
                    file_lines[file_operator['mode_value'][0] - 1])
            if file_operator['mode'] == 'list':
                for line in file_operator['mode_value']:
                    file_lines[line - 1] = comment_operation(file_lines[line - 1])
            if file_operator['mode'] == 'section':
                for line in range(file_operator['mode_value'][0] - 1, file_operator['mode_value'][1]):
                    file_lines[line] = comment_operation(file_lines[line])
        do_comment_in_lines_mode(file_operator, file_lines)

        with open(file_operator['file'], 'w') as file_open:
            file_open.writelines(file_lines)
    elif file_operator['point'] == 'content':
        file_contents = None
        with open(file_operator['file'], 'r') as file_open:
            file_contents = str(file_open.read())

        def do_comment_in_content_mode(file_operator, file_contents):
            """do_comment_in_content_mode"""
            if file_operator['mode'] == 'first':
                file_contents = file_contents.replace(file_operator['mode_value'], \
                    '"""{}"""'.format(file_operator['mode_value']), 1)
            if file_operator['mode'] == 'all':
                file_contents = file_contents.replace(file_operator['mode_value'], \
                    '"""{}"""'.format(file_operator['mode_value']))
        do_comment_in_content_mode(file_operator, file_contents)
        with open(file_operator['file'], 'w') as file_open:
            file_open.write(file_contents)


def uncomment_operator(file_operator):
    """uncomment operation"""
    if file_operator['point'] == 'lines':
        file_lines = None
        with open(file_operator['file'], 'r') as file_open:
            file_lines = file_open.readlines()
        for line in file_operator['mode_value']:
            if line >= len(file_lines):
                raise ValueError('Specified line {} is out of range'.format(line))

        def do_uncomment_operator(file_operator, file_lines):
            """do_uncomment_operator"""
            if file_operator['mode'] == 'single':
                file_lines[file_operator['mode_value'][0] - 1] = \
                    file_lines[file_operator['mode_value'][0] - 1].replace('# ', '', 1)
            if file_operator['mode'] == 'list':
                for line in file_operator['mode_value']:
                    file_lines[line - 1] = file_lines[line - 1].replace('# ', '', 1)
            if file_operator['mode'] == 'section':
                for line in range(file_operator['mode_value'][0] - 1, file_operator['mode_value'][1]):
                    file_lines[line] = file_lines[line].replace('# ', '', 1)
        do_uncomment_operator(file_operator, file_lines)
        with open(file_operator['file'], 'w') as file_open:
            file_open.writelines(file_lines)


def do_operation(file_operator):
    """excute file operation"""
    operators = {
        'replace': replace_operator,
        'delete': delete_operator,
        'insert': insert_operator,
        'comment': comment_operator,
        'uncomment': uncomment_operator
    }
    operators[file_operator['operator']](file_operator)


if __name__ == '__main__':
    FILE_OPERATION = args_check(OPERATOR_ARGS)
    print(FILE_OPERATION)
    do_operation(FILE_OPERATION)
