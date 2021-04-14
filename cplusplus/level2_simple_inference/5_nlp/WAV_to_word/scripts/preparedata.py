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
    data_input = np.zeros((range0_end, 200), dtype=np.float64)  # 用于存放最终的频率特征数据
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
    print(out_filename)
    writer.write(features)
    print("save success")
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

if __name__ == "__main__":
    current_path = os.path.abspath(__file__)    # 获取当前文件的父目录
    voicefiles = os.listdir(r'../data/') # 获取wav
    for voice_name in voicefiles:
        if not voice_name.endswith("wav"):
            continue
        print("start to process image {}....".format(voice_name))
        inputname = os.path.join(os.path.abspath(os.path.dirname(current_path) + os.path.sep + "../data"),voice_name)
        GetDataSet(inputname)
    #in_len = GetDataSet("../data/1.wav")
    #resultList = np.fromfile("F:\\202007\\cjl\out\\20200704_142002_0\\voice_output_0.bin",np.float32)

    # 判断模型推理结果是否成功
    #if resultList is None:
       # print("Inference failed")
    #resultList=np.reshape(resultList,(200,1424))
    # 对结果进行后处理
    #txt, pinyin = SpeechPostProcess(resultList,in_len)

    #print('拼音： ' + str(pinyin))
    #print('文本： ' + str(txt))
