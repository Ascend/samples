# -*- coding: UTF-8 -*-
#包的引入 这些包python自带
import os
from subprocess import call
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--smile_file", default="")
parser.add_argument("--config_file", default="")
parser.add_argument("--audio_path", default="")
parser.add_argument("--output_path", default="")
args = parser.parse_args()

#路径设置
#SMILExtract所在的文件路径
pathExcuteFile = args.smile_file
#opensmile配置文件所在的路径  选用emobase2010.conf路径
pathConfig = args.config_file
pathAudio = args.audio_path
pathOutput = args.output_path
#利用cmd调用exe文件
def excuteCMD(_pathExcuteFile,_pathConfig,_pathAudio,_pathOutput,_Name):
    cmd = _pathExcuteFile + " -C " + _pathConfig + " -I " + _pathAudio + " -O " + _pathOutput + " -N " + _Name
    call(cmd, shell=True)

def loopExcute(pathwav,patharff):
    i = 0 # 子目录，对目录里所有wav目标文件进行处理
    wavlist = os.listdir(pathwav)
    wavpath = sorted(wavlist)
    clearDataout = "rm -rf " + patharff + "*"
    call(clearDataout, shell=True)
    for category in wavpath:
        i = i + 1
        #print(category)
        category_path = os.path.join(pathwav,category)
        print(category_path)
        wavfiles = os.listdir(category_path)
        wavfilelist = sorted(wavfiles)
        for files in wavfilelist:
            #print(files)
            print(files)
            file_path = os.path.join(category_path,files)
            name = str(i) + '-' + os.path.splitext(file_path)[0]
            outputname = 'audio_feature0{}.txt'.format(i)
            output_path = os.path.join(patharff,outputname)
            excuteCMD(pathExcuteFile, pathConfig, file_path, output_path, name)

if __name__ == '__main__':
    loopExcute(pathAudio, pathOutput)