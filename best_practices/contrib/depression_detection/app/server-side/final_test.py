# -*- coding: UTF-8 -*-
#包的引入 这些包python自带
import os
from subprocess import call
import sys

sys.path.append("../../../../../python/common/")
sys.path.append("../../../../../python/common/acllite")
sys.path.append("../")

#path = os.path.dirname(os.path.abspath(__file__))
#sys.path.append(os.path.join(path, "samples/python/common/acllite"))
from django.http import response, HttpResponse, JsonResponse
from django.shortcuts import render
import wave
import numpy as np

import acl
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
import ast
from constants import *

#路径设置
path = os.getcwd().replace('\\','/')
#SMILExtract所在的文件路径
pathExcuteFile = '/home/audioprocessing_linux/opensmile/bin/linux_x64_standalone_libstdc6/SMILExtract'
#opensmile配置文件所在的路径
pathConfig = '/home/audioprocessing_linux/opensmile/config/emobase2010.conf'
pathAudio = os.path.join(path, 'static', 'audios') 
pathOutput = os.path.join(path, 'static')
#利用cmd调用exe文件
def excuteCMD(_pathExcuteFile,_pathConfig,_pathAudio,_pathOutput,_Name):
    cmd = _pathExcuteFile + " -C " + _pathConfig + " -I " + _pathAudio + " -O " + _pathOutput + " -N " + _Name
    call(cmd, shell=True)

def aac2raw(aac_path,raw_path):
    cmd = 'ffmpeg -i ' + aac_path + ' -f s16le -acodec pcm_s16le -ar 44100 -ac 1 ' + raw_path
    call(cmd, shell=True)

def loopExcute(pathwav,patharff):
    for file in os.listdir(pathwav):
        if os.path.splitext(file)[1] == '.wav':
            #print(file)
            file_path = os.path.join(pathwav,file)
            name = os.path.splitext(file)[0]
            outputname = 'audio_feature.txt'
            output_path = os.path.join(patharff,outputname)
            excuteCMD(pathExcuteFile, pathConfig, file_path, output_path, name)

def readFile(path):
    f = open(path)
    lines = f.readlines()
    del lines[0:1589]
    first_ele = True
    for data in lines:
        data = data.strip('\n')
        nums = data.split(',')
        if first_ele:
            matrix = np.array(nums)
            first_ele = False
        else:
            matrix = np.c_[matrix,nums]
    matrix = matrix.transpose()
    a = []
    b = [227,1482,459,176,328,1492,1130,117,1472,621,416,276,365,983,1244,879,795,507,1514,612]
    for x in range(0,3):
        result = [float(matrix[x][c]) for c in b]
        a.append(result)
    arr=np.array(a)
    f.close()
    return arr

ii = -1
i = 0

acl_resource = AclLiteResource()
acl_resource.init()
cont, ret = acl.rt.get_context()

def test(request):
    global i,ii
    context = {}
    context['msg'] = ''
    context['result'] = ''
    j = 0
    if request.method == 'POST':
        ret = acl.rt.set_context(cont)
        files = request.FILES.getlist('file')
        ii += 1
        print(files)
        for f in files:
            j += 1
            print(f)
            filename1 = '0{}'.format(ii%3+1) + '.aac'
            filename2 = '0{}'.format(ii%3+1) + '.raw'
            wavfilename = '0{}'.format(ii%3+1) + '.wav'
            f_audio = open(os.path.join(pathAudio,filename1),'wb+')
            for chunk in f.chunks():
                f_audio.write(chunk)
            f_audio.close()
            
            aac_path = os.path.join(pathAudio,filename1)
            raw_path = os.path.join(pathAudio,filename2)
            aac2raw(aac_path,raw_path)
        
            pcmfile = open(os.path.join(pathAudio,filename2), 'rb')
            pcmdata = pcmfile.read()
            wavfile = wave.open(os.path.join(pathAudio,wavfilename), 'wb')
            wavfile.setframerate(44100)
            wavfile.setsampwidth(2)    #24位采样即为3字节
            wavfile.setnchannels(1)
            wavfile.writeframes(pcmdata)
            wavfile.close()
        
        loopExcute(pathAudio, pathOutput)
        for file in os.listdir(pathAudio):
            os.remove(os.path.join(pathAudio,file))
        i += 1
        if i%3 == 0:
            x = readFile(os.path.join(path, 'static', 'audio_feature.txt'))
            x = x[:, 0:20]
            for m in range(0, 3):
                for n in range(0, 20):
                    if float(x[m][n]) != 0.0:
                        x[m][n] = float(format(x[m][n] + 0.000000000000001, '.3g'))
     
            health = 0
            depression = 0
            for idx in range(1,4):
                data_arr = np.array(x[idx-1], dtype = np.float32)
                input_blob = np.expand_dims(data_arr,axis=0).astype(np.float32)
                input_blob = input_blob.reshape(1,1,20,1)
                model = AclLiteModel(r'/home/wangxu/tf_model_{}.om'.format(idx))
                output = model.execute(input_blob)
                print(output)
                out = np.array(output[0][0])
                if int(np.argmax(out,axis=0)) == 0:
                    health += 1
                else :
                    depression += 1
            
            context['msg'] = 'ok'
            
            if health >= 2:
                context['result'] = "健康"
            elif depression >= 2:
                context['result'] = "抑郁"
       
            os.remove(os.path.join(path, 'static', 'audio_feature.txt'))
            print(context['result'])
    return JsonResponse(context)


def result(request):
    return render(request, "result.html")