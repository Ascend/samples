# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

# coding=utf-8

#pcm_path = r'speech_voice/01.pcm'
import os
import numpy as np
import wave
from scipy.fftpack import fft
from ctc_func import greedy_decode
from get_symbol_list import GetSymbolList
from language_model_func import ModelLanguage

x=np.linspace(0, 400 - 1, 400, dtype = np.int64)
w = 0.54 - 0.46 * np.cos(2 * np.pi * (x) / (400 - 1) ) # 汉明窗
AUDIO_FEATURE_LENGTH = 200
outpath = "/mnt/shared/project/NLP/out/"
def pcm2wav(pcm_path):
    # 打开并去读pcm音频
    pcmfile = open(pcm_path, 'rb')
    pcmdata = pcmfile.read()
    pcmfile.close()

    # 设置wav 音频参数
    channels = 2
    bits = 16
    sample_rate = 16000

    # 定义wav音频的生成路径和名称
    wave_path_front = pcm_path[:-4]
    wave_path = wave_path_front + '.wav'

    # 创建wav音频文件
    wavfile = wave.open(wave_path, 'wb')

    wavfile.setnchannels(channels)
    wavfile.setsampwidth(bits // 8)
    wavfile.setframerate(sample_rate)

    # 写入wav音频数据
    wavfile.writeframes(pcmdata)

    wavfile.close()

    return wave_path
def read_wav_data(filename):
    wav=wave.open(filename,"rb")
    num_frame=wav.getnframes()
    num_channel=wav.getnchannels()
    framerate=wav.getframerate()
    num_sample_width=wav.getsampwidth()
    str_data=wav.readframes(num_frame)
    wav.close()
    wave_data=np.frombuffer(str_data,dtype=np.short)
    wave_data.shape=-1,num_channel
    wave_data=wave_data.T
    #print("ks",framerate)
    return wave_data,framerate

def GetFrequencyFeature3(wavsignal, fs):
    if (16000 != fs):
        raise ValueError(
            '[Error] ASRT currently only supports wav audio files with a sampling rate of 16000 Hz, but this audio is ' + str(
                fs) + ' Hz. ')

    # wav波形 加时间窗以及时移10ms
    time_window = 25  # 单位ms
    window_length = fs / 1000 * time_window  # 计算窗长度的公式，目前全部为400固定值
    #print window_length
    wav_arr = np.array(wavsignal)
    # wav_length = len(wavsignal[0])
    wav_length = wav_arr.shape[1]
    range0_end = int(float(len(wavsignal[0])) / fs * 1000 - time_window) // 10  # 计算循环终止的位置，也就是最终生成的窗数 978
    data_input = np.zeros((range0_end, 200), dtype=np.float)  # 用于存放最终的频率特征数据
    data_line = np.zeros((1, 400), dtype=np.float)
    for i in range(0, range0_end):
        p_start = i * 160
        p_end = p_start + 400
        data_line = wav_arr[0, p_start:p_end]
        data_line = data_line * w  # 加窗
        data_line = np.abs(fft(data_line)) / wav_length
        data_input[i] = data_line[0:200]  # 设置为400除以2的值（即200）是取一半数据，因为是对称的

    # print(data_input.shape)
    data_input = np.log(data_input + 1)
    return data_input

def RecognizeSpeech(wavsignal, fs):
    data_input = GetFrequencyFeature3(wavsignal, fs)
    input_length = len(data_input)  #978
    input_length = input_length // 8  #122

    data_input = np.array(data_input, dtype=np.float32)

    data_input = data_input.reshape(data_input.shape[0], data_input.shape[1], 1)  #978,200,1
    batch_size = 1
    in_len = np.zeros((batch_size), dtype = np.int32)

    in_len[0] = input_length

    x_in = np.zeros((batch_size, 1600, AUDIO_FEATURE_LENGTH, 1), dtype=np.float32) #1,1600,200,1

    for i in range(batch_size):
        x_in[i, 0:len(data_input)] = data_input

    return x_in, in_len

def RecognizeSpeech_FromFile(filename):
    '''
    最终做语音识别用的函数，识别指定文件名的语音
    '''

    wavsignal,fs1 = read_wav_data(filename)  # 识别语音的特征 fs1=16000 len(wavsignal[0])=157000
    r, in_len = RecognizeSpeech(wavsignal, fs1)
    return r, in_len
def GetDataSet(speech_voice_path):
    """ 读取pcm格式音频数据 """

    # 将pcm数据转换为wav
    #wave_path = L.pcm2wav(speech_voice_path) 

    # 读取wav音频特征
    features, in_len = RecognizeSpeech_FromFile(speech_voice_path)     

    # 将wav音频特征转换为模型输入向量
    out_file_name = speech_voice_path.split('.')[0]
    out_filename = out_file_name+'.bin'
    writer = open(out_filename,"wb")
    writer.write(features)
    return in_len

def GetDataSet2(speech_voice_path):
    """ 直接读取wav格式音频数据 """

    features, in_len = RecognizeSpeech_FromFile(speech_voice_path) #1,1600,200,1  in_len=122 全0矩阵
    features1=np.reshape(features,[1,1600,200,1])

    features1=np.transpose(features1,(0,3,1,2)).copy()
    np.save('features1',features1)
    
    writer = open("features1.bin","wb")
    writer.write(features)
    return  in_len

def SpeechPostProcess(resultList, in_len): 

    # 将三维矩阵转为二维
    dets = np.reshape(resultList, (200,1424))

    # 将识别结果转为拼音序列
    rr, ret1 = greedy_decode(dets)

    # 去除拼音序列中的blank
    for i in range(len(ret1)):
        if i % 2 == 0:
            try:
                ret1.remove(1423)
            except:
                pass

    list_symbol_dic = GetSymbolList()

    r_str = []
    for i in ret1:
        r_str.append(list_symbol_dic[i])

    #print "拼音序列识别结果：" + str(r_str)
    string_pinyin = str(r_str)

    ml = ModelLanguage('language_model')

    ml.LoadModel()

    str_pinyin = r_str

    r = ml.SpeechToText(str_pinyin)

    print(r)

    # 保存语音识别的结果
    with open('results/asr_results.txt','a+b') as f:
        data = string_pinyin[1:-1] + '-' + r + '\n'
        #print(data)
        data=data.encode()
        f.write(data)
        f.close()

    return r, str_pinyin

dict = {'nihao.wav':'output1_0.bin','xinpian.wav':'output2_0.bin'}
if __name__ == "__main__":

    current_path = os.path.abspath(__file__)    # 获取当前文件的父目录
    voicefiles = os.listdir(r'../data/') # 获取wav
    for voice_name in voicefiles:
        if not voice_name.endswith("nihao.wav"):
            continue
        print("start to process image {}....".format(voice_name))
        inputname = os.path.join(os.path.abspath(os.path.dirname(current_path) + os.path.sep + "../data/"),voice_name)
        in_len = GetDataSet(inputname)

        outputname = os.path.join(os.path.abspath(os.path.dirname(current_path) + os.path.sep + "../out/"),dict[voice_name])
        resultList = np.fromfile(outputname,np.float32)
        # 判断模型推理结果是否成功
    	#if resultList is None:
            #print("Inference failed")
        resultList=np.reshape(resultList,(200,1424))
        # 对结果进行后处理

        txt, pinyin = SpeechPostProcess(resultList,in_len)
        print('拼音： ' + str(pinyin))
        print('文本： ' + str(txt))