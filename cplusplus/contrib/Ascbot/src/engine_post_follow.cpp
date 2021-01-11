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
#include "engine_post_follow.h"
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

namespace {

int wheel_speed_offset = 0;  //(-2----5)
int uart_print_count = 0;

struct shared_common_st *shared_stuff;
void *shared_memory = (void *)0;
int shmid;

struct PostData post_data;
}

int EnginePostFollow::_s_flag = 0;
// constants
EnginePostFollow::EnginePostFollow() {

}

int EnginePostFollow::common_shm_init(void)
{
    INFO_LOG("common_shm_init FOLLOW------------ \n");
    shmid = shmget((key_t)1234, sizeof(struct shared_common_st), 0666 | IPC_CREAT);

    if (shmid == -1) {
        INFO_LOG("shmget failed 1\n");
        return -1;
    }

    shared_memory = shmat(shmid, (void *)0, 0);

    if (shared_memory == (void *)-1) {
        INFO_LOG("shmget failed 2\n");
        return -1;
    }

    shared_stuff = (struct shared_common_st *)shared_memory;
    shared_stuff->written_flag = 0;
    shared_stuff->wheel_speed_written_flag = 0;

    return 0;
}

int EnginePostFollow::Init() {

    common_shm_init();
    post_data.angle = 0;
    post_data.direction = 0;
    post_data.work_mode = 0;

    INFO_LOG("EnginePostFollow End initialize!\n");
    return 0;
}

int EnginePostFollow::hand_wheel(void)
{
    if(post_data.work_mode==ASCBOT_OBJECT_MODE)
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

        if(post_data.work_mode == ASCBOT_OBJECT_MODE)
             wheel_speed_offset = 1;
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
    INFO_LOG("follow post_data.wheel_left_speed:%d  post_data.wheel_right_speed:%d\n",post_data.wheel_left_speed,post_data.wheel_right_speed);
    INFO_LOG("follow shared_stuff->wheel_left_speed:%d  shared_stuff->wheel_right_speed:%d\n",shared_stuff->wheel_left_speed,shared_stuff->wheel_right_speed);

    return 0;
}

int EnginePostFollow::Handle_off_mode(float *result,int size)
{
    post_data.angle = 0;
    post_data.direction = 0;
    return 0;
}

int EnginePostFollow::Handle_follow_mode(float *result,int size)
{
    INFO_LOG("EnginePostFollow:  %f %f  %f %f \n",result[1] ,result[2] ,result[3],result[5]);
    if(result[3] < 0.0){
        result[3] = 0.0;
    }

    if(result[5] < 0){
        result[5] = 1.0;
    }

    float center_xpos = (result[3] + result[5]) / 2;
    if((result[1] == 5) && (result[2] > 0.9) && (result[3] < 0.9) && (result[5] > 0.1))
    {
        if(center_xpos < 0.3)
        {
            post_data.angle = 45;
            post_data.direction = 1;
        }
        else if(center_xpos < 0.4)
        {
            post_data.angle = 30;
            post_data.direction = 1;
        }
        else if(center_xpos < 0.45)
        {
            post_data.angle = 15;
            post_data.direction = 1;
        }
        else if(center_xpos < 0.49)
        {
            post_data.angle = 5;
            post_data.direction = 1;
        }
         else if(center_xpos > 0.7)
        {
             post_data.angle = -45;
             post_data.direction = 1;
        }
        else if(center_xpos > 0.6)
        {
            post_data.angle = -30;
            post_data.direction = 1;
        }
        else if(center_xpos > 0.55)
        {
            post_data.angle = -15;
            post_data.direction = 1;
        }
        else if(center_xpos > 0.51)
        {
             post_data.angle = -5;
             post_data.direction = 1;
        }
        else
        {
             post_data.angle = 0;
             post_data.direction = 1;
        }
    }
    else
    {
        Handle_off_mode(result,size);
    }

    return 0;
}

int EnginePostFollow::Handle_object_mode(float *result,int size)
{
    if(((result[1]== 5)||(result[1]==6))&&(result[2]>0.5)&&(result[3]<0.9)&&(result[5]>0.1))
    {
        shared_stuff->wheel_force_stop = 1;
    }
    else
    {
        shared_stuff->wheel_force_stop = 0;
    }

    return 0;
}

int EnginePostFollow::HandleResults(float *result,int size, int mode) {
    post_data.work_mode = mode;
    INFO_LOG(" enter post follow\n");
    if (mode != ASCBOT_OBJECT_MODE) {
        return 0;
    }
    Handle_follow_mode(result,size);
    hand_wheel();
    return 0;
}

int EnginePostFollow::handle_preview(ImageData& result,int mode)
{
    if (mode != ASCBOT_ROAD_MODE) {
        return -1;
    }

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