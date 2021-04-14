/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include "engine_post_dange.h"
#include <cstdio>
#include <ios>
#include <vector>
#include <sstream>
#include <cmath>
#include <regex>

#include <thread>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>

#include <sys/shm.h>

#include "uart_print.h"
#include "common_post.h"
using namespace std;

#include "engine_handle.h"
#include "utils.h"

namespace {


int wheel_speed_offset = 0;  //(-2----5)

int uart_print_count = 0;

struct shared_common_st *shared_stuff;
void *shared_memory = (void *)0;
int shmid;

struct PostData post_data;
}

int EnginePostDange::_s_flag = 0;
// constants
EnginePostDange::EnginePostDange() {

}

int EnginePostDange::common_shm_init(void)
{
  INFO_LOG("common_shm_init DANGE------------ \n");
  shmid = shmget((key_t)1234, sizeof(struct shared_common_st), 0666 | IPC_CREAT);
  if (shmid == -1) {
      ERROR_LOG("shmget failed 1\n");
    return -1;
  }

  shared_memory = shmat(shmid, (void *)0, 0);

  if (shared_memory == (void *)-1) {
      ERROR_LOG("shmget failed 2\n");
    return -1;
  }

  shared_stuff = (struct shared_common_st *)shared_memory;
  shared_stuff->written_flag = 0;
  shared_stuff->wheel_speed_written_flag = 0;

  return 0;
}

int EnginePostDange::Init() {

  INFO_LOG("EnginePostDange Begin initialize!\n");
  common_shm_init();

  post_data.angle = 0;
  post_data.direction = 0;
  post_data.work_mode = 0;

  INFO_LOG( "EnginePostDange End initialize!\n");
  return 0;
}


int EnginePostDange::hand_wheel(void)
{
    if((post_data.work_mode== ASCBOT_VOID_DANGER_MODE)||(post_data.work_mode==ASCBOT_OFF))
    {
        if(post_data.angle == 180 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = -0x07;
            post_data.wheel_right_speed = 0x07;
        }
        else  if(post_data.angle == -180 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x07;
            post_data.wheel_right_speed = -0x07;
        }
        else  if(post_data.angle == 0 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x07;
            post_data.wheel_right_speed = 0x07;
        }
        else  if(post_data.angle >= 45 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x03;
            post_data.wheel_right_speed = 0x08;
        }
        else  if(post_data.angle >= 30 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x04;
            post_data.wheel_right_speed = 0x07;
        }
        else  if(post_data.angle >= 15 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x05;
            post_data.wheel_right_speed = 0x07;
        }
        else  if(post_data.angle >= 5 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x06;
            post_data.wheel_right_speed = 0x07;
        }
        else  if(post_data.angle <= -45 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x08;
            post_data.wheel_right_speed = 0x03;
        }
        else  if(post_data.angle <= -30 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x07;
            post_data.wheel_right_speed = 0x04;
        }
        else  if(post_data.angle <= -15 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x07;
            post_data.wheel_right_speed = 0x05;
        }
        else  if(post_data.angle <= -5 && post_data.direction == 1)
        {
            post_data.wheel_left_speed = 0x07;
            post_data.wheel_right_speed = 0x06;
        }
        else  if(post_data.direction == 0)
        {
            post_data.wheel_left_speed = 0;
            post_data.wheel_right_speed = 0;
        }


        if(post_data.work_mode == ASCBOT_VOID_DANGER_MODE)
             wheel_speed_offset = 0;
        else
             wheel_speed_offset = 0;

        //set shared memory
        if(shared_stuff->wheel_speed_written_flag == 0)
        {
            shared_stuff->wheel_left_speed = post_data.wheel_left_speed + wheel_speed_offset;
            shared_stuff->wheel_right_speed = post_data.wheel_right_speed + wheel_speed_offset;
            shared_stuff->wheel_speed_written_flag = 1;
        }

    }
    INFO_LOG("dange post_data.wheel_left_speed:%d  post_data.wheel_right_speed:%d\n",post_data.wheel_left_speed,post_data.wheel_right_speed);
    INFO_LOG("dange shared_stuff->wheel_left_speed:%d  shared_stuff->wheel_right_speed:%d\n",shared_stuff->wheel_left_speed,shared_stuff->wheel_right_speed);
    return 0;
}


int EnginePostDange::Handle_dange_mode(float *result,int size)
{
    if((result[1]<0.9)&&(result[0]>0.8))
    {
        post_data.angle = -180;
        post_data.direction = 1;
    }
    else if(result[1]>0.999)
    {
        post_data.angle = 0;
        post_data.direction = 1;
    }

    INFO_LOG("post_data.angle:%d  post_data.direction:%d\n",post_data.angle,post_data.direction);
    return 0;
}

int EnginePostDange::Handle_off_mode(float *result,int size)
{
     post_data.angle = 0;
     post_data.direction = 0;
     return 0;
}

int EnginePostDange::Handle_remote_mode(float *result,int size)
{
    return 0;
}

int EnginePostDange::HandleResults(float* Pdata, int size,int mode) {
	
	post_data.work_mode = mode;
    switch(mode)
    {
        case ASCBOT_OFF:
            Handle_off_mode(Pdata,size);
            break;
        case ASCBOT_VOID_DANGER_MODE:
            Handle_dange_mode(Pdata,size);
            break;
        case ASCBOT_REMOTE_MODE:
            Handle_remote_mode(Pdata,size);
            break;
        default:
            Handle_off_mode(Pdata,size);
            break;
    }
    hand_wheel();
    return 0;
}
int EnginePostDange::handle_preview(ImageData& result,int mode)
{
    if(mode!=ASCBOT_ROAD_MODE)
        return -1;

    if (shared_stuff->image_data_written_flag == 0)
    {
        memcpy(shared_stuff->image_data, result.data.get(),result.size);
        shared_stuff->image_data_size = result.size;
        shared_stuff->image_data_width = 256;
        shared_stuff->image_data_height = 256;
        shared_stuff->image_data_written_flag = 1;
    }
    return 0;
}
