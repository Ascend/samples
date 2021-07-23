"""
# coding=utf-8
#import platform as plat
"""

import os
def GetSymbolList():
    """
    Function description:
    Processing sequence
    Parameter:
    ...
    Return Value:
    list_symbol
"""
    current_path = os.path.dirname(__file__)
    txt_obj = open(os.path.join(current_path + "/dict.txt"), 'r', encoding='UTF-8')
    txt_text = txt_obj.read()
    #print(txt_text)
    txt_lines = txt_text.split('\n')
    list_symbol = []

    for i in txt_lines:
        if (i != ''):
            txt_l = i.split('\t')
            list_symbol.append(txt_l[0])

    txt_obj.close()
    list_symbol.append('_')

    return list_symbol
