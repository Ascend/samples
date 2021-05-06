/**
* @file uart.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>

#include "mic.h"
#include <termios.h>    /*PPSIX*/

Mic::Mic(void)
{
    int ret = MediaLibInit();
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("MediaLibInit failed! ");
    }
    else
    {
        INFO_LOG("MediaLibInit  success! ");
    }
}

Mic::~Mic(void)
{

}

int Mic::mic_open(void)
{
    int ret = OpenMIC();
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("mic open  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("mic open  success! ");
    }
    return ASCEND_MIC_SUCCESS;
}

int Mic::mic_close(void)
{
    int ret = CloseMIC();
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("mic close  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("mic close  success! ");
    }
    return ASCEND_MIC_SUCCESS;

}

int Mic::mic_qry_status()
{
    int ret = QueryMICStatus();
    switch(ret)
    {
        case MIC_STATUS_OPEN:
        {
            INFO_LOG("QueryMICStatus  MIC_STATUS_OPEN! ");
            break;
        }
        case MIC_STATUS_CLOSED:
        {
            INFO_LOG("QueryMICStatus  MIC_STATUS_CLOSED! ");
            break;
        }
        case MIC_NOT_EXISTS:
        {
            INFO_LOG("QueryMICStatus  MIC_NOT_EXISTS! ");
            break;
        }
        default:
        {
            INFO_LOG("QueryMICStatus  MIC_STATUS_UNKOWN! ");
            break;
        }
    }
    return ret;
}

int Mic::mic_get_property(struct MICProperties *propties)
{
    if(nullptr  == propties)
    {
        ERROR_LOG("mic_get_property  param nullptr! ");
        return ASCEND_MIC_ERROR;
    }
    int ret = GetMICProperty(propties);
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("mic_get_property  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("mic_get_property  success! ");
    }
    return ASCEND_MIC_SUCCESS;
}
int Mic::mic_set_property(MICProperties *propties)
{
    if(nullptr  == propties)
    {
        ERROR_LOG("mic_set_property  param nullptr! ");
        return ASCEND_MIC_ERROR;
    }
    if((MIC_AUDIO_SAMPLE_RATE_8000 != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_12000 != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_11025  != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_16000 != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_22050 != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_24000  != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_32000  != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_44100  != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_48000  != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_64000 != propties->sample_rate) &&
        (MIC_AUDIO_SAMPLE_RATE_96000  != propties->sample_rate))
    {
        ERROR_LOG("mic_set_property   param sample_rate error %d!", propties->sample_rate);
        return ASCEND_MIC_ERROR;
    }
    if((MIC_CAP_ACTIVE != propties->cap_mode) &&
        (MIC_CAP_PASSIVE != propties->cap_mode))
    {
        ERROR_LOG("mic_set_property   param cap_mode error %d!", propties->cap_mode);
        return ASCEND_MIC_ERROR;
    }
    if((MIC_AUDIO_BIT_WIDTH_16  != propties->bit_width) &&
        (MIC_AUDIO_BIT_WIDTH_24  != propties->bit_width))
    {
        ERROR_LOG("mic_set_property   param bit_width error %d!", propties->bit_width);
        return ASCEND_MIC_ERROR;
    }
    if((MIC_SAMPLE_NUM_80  != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_160  != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_240  != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_320 != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_480  != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_1024 != propties->frame_sample_rate) &&
        (MIC_SAMPLE_NUM_2048  != propties->frame_sample_rate))
    {
        ERROR_LOG("mic_set_property  param frame_sample_rate error %d!", propties->frame_sample_rate);
        return ASCEND_MIC_ERROR;
    }

    if((MIC_AUDIO_SOUND_MODE_MONO  != propties->sound_mode) &&
        (MIC_AUDIO_SOUND_MODE_STEREO  != propties->sound_mode))
    {
        ERROR_LOG("mic_set_property  param sound_mode error %d! ",propties->sound_mode);
        return ASCEND_MIC_ERROR;
    }

    int ret = SetMICProperty(propties);
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("mic_set_property  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("mic_set_property  success! ");
    }
    return ASCEND_MIC_SUCCESS;
}
int Mic::mic_cap(CAP_MIC_CALLBACK tfunc, void* param)
{
    int ret = CapMIC(tfunc, param);
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("CapMIC  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("CapMIC  success! ");
    }
    return ASCEND_MIC_SUCCESS;
}
int Mic::mic_read_sound(void* pdata, int *size)
{
    if((nullptr == pdata) || (nullptr == size))
    {
        ERROR_LOG("mic_read_sound  param nullptr! ");
        return ASCEND_MIC_ERROR;
    }
    int ret = ReadMicSound(pdata, size);
    if(LIBMEDIA_STATUS_FAILED == ret)
    {
        ERROR_LOG("ReadMicSound  failed! ");
        return ASCEND_MIC_ERROR;
    }
    else
    {
        INFO_LOG("ReadMicSound  success! ");
    }
    return ASCEND_MIC_SUCCESS;
}



