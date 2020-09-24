# coding=utf-8
#import platform as plat

def GetSymbolList():
    txt_obj = open('dict.txt', 'r',encoding='UTF-8')
    txt_text = txt_obj.read()
    #print(txt_text)
    txt_lines = txt_text.split('\n')
    list_symbol = []

    for i in txt_lines:
        if (i!=''):
            txt_l = i.split('\t')
            list_symbol.append(txt_l[0])

    txt_obj.close()
    list_symbol.append('_')

    return list_symbol
