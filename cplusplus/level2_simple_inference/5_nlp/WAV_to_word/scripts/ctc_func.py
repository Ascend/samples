# coding=utf-8
""" 构建CTC解码函数 """

import numpy as np 

#以下函数为贪婪算法的python实现
def remove_blank(labels, blank=0):
    """ 移除Blank """
    new_labels = []

    # combine duplicate
    previous = None
    for l in labels:
        if l != previous:
            new_labels.append(l)
            previous = l

    # remove blank
    new_labels = [l for l in new_labels if l != blank]

    return new_labels

def insert_blank(labels, blank=0):
    """ 插入blank """
    new_labels = [blank]
    for l in labels:
        new_labels += [l, blank]

    return new_labels

def greedy_decode(y, blank=0):
    """ greedy解码 """
    raw_rs = np.argmax(y, axis=1)
    rs = remove_blank(raw_rs, blank)
    
    return raw_rs, rs

