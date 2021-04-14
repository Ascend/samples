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
#include "engine_handle.h"
#include <ios>
#include <vector>
#include <sstream>
#include <cmath>
#include <regex>
//#include "hiaiengine/log.h"

#include <thread>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>

#include <sys/shm.h>

#include "uart_print.h"
#include "tcp_server.h"
#include "wheel.h"
#include "i2c.h"
#include "oled.h"
#include "common_post.h"
#include "utils.h"
using namespace std;

namespace {

    char mode_off[20] = "OFF";
    char mode_dange[20] = "Avoid Dange";
    char mode_follow[20] = "Follow Object";
    char mode_road[20] = "Road Run";
    char mode_remote[20] = "Remote";
    char getting_ip[20] = "getting ip";

    int uart_print_count = 0;
    oled oled_ctrl;

    struct shared_common_st* shared_stuff;

    struct PostData post_data;

    void* shared_memory = (void*)0;
    int shmid;

    int current_work_mode = ASCBOT_ROAD_MODE;//ASCBOT_ROAD_MODE  ASCBOT_OFF

    int current_remote_left_speed = 0;
    int current_remote_right_speed = 0;
    int current_remote_direction = 0;

    int current_wheel_left_speed = 0;
    int current_wheel_right_speed = 0;

}

// constants
EngineHandle::EngineHandle() {

}

int EngineHandle::common_shm_init(void)
{
    INFO_LOG("common_shm_init_handle------------ \n");
    shmid = shmget((key_t)1234, sizeof(struct shared_common_st), 0666 | IPC_CREAT);

    if (shmid == -1) {
        ERROR_LOG("shmget failed\n");
        return -1;
    }
     shared_memory = shmat(shmid, (void *)0, 0);
     if (shared_memory == (void *)-1) {
         ERROR_LOG("shmat failed\n");
        return -1;
    }
    shared_stuff = (struct shared_common_st *)shared_memory;
    shared_stuff->written_flag = 0;
    return 0;
}



int EngineHandle::common_shm_process_handle(void)
{
    INFO_LOG(" shared_stuff->written_flag:%d shared_stuff->wheel_speed_written_flag:%d\n", shared_stuff->written_flag,shared_stuff->wheel_speed_written_flag);
    if (shared_stuff->written_flag == 1)
    {
        post_data.work_mode = shared_stuff->work_mode;  //get work_mode from APK by tcp
        post_data.remote_direction = shared_stuff->remote_direction;
        post_data.remote_left_speed = shared_stuff->remote_left_speed;
        post_data.remote_right_speed = shared_stuff->remote_right_speed;
        shared_stuff->written_flag = 0;
    }

    if (shared_stuff->wheel_speed_written_flag == 1)
    {
        post_data.wheel_left_speed =  shared_stuff->wheel_left_speed;
        post_data.wheel_right_speed =  shared_stuff->wheel_right_speed;
        shared_stuff->wheel_speed_written_flag = 0;

        if(shared_stuff->wheel_force_stop == 1)  //check road mode
        {
            post_data.wheel_left_speed = 0;
            post_data.wheel_right_speed = 0;
        }
    }

    if(shared_stuff->ip_written_flag == 1)
    {
        strncpy(post_data.ip_address , shared_stuff->ip_address,20);
        shared_stuff->ip_written_flag = 0;
        post_data.ip_get_flag = 1;
        ERROR_LOG("post_data.ip_address------------- %s  \n",post_data.ip_address);
    }
    INFO_LOG("left_speed:%d,right_speed:%d\n",post_data.wheel_left_speed,post_data.wheel_right_speed);
    return 0;
}

int EngineHandle::Init( ) {
    INFO_LOG("Begin EngineHandle initialize!");
    tcp_server tcp_server_ctrl;
    tcp_server_ctrl.tcp_server_create();
    wheel_ctrl.wheel_init();

    common_shm_init();

    oled_ctrl.OLED_Init();
    oled_ctrl.OLED_CLS_RAM();

    oled_ctrl.OLED_ShowStr(0, 0, getting_ip, 1);
    oled_ctrl.OLED_ShowStr(1, 0, mode_off, 1);
    oled_ctrl.OLED_Screen_display();

    post_data.angle = 0;
    post_data.direction = 0;
    post_data.work_mode = 0;
    post_data.ip_get_flag = 0;

    INFO_LOG("End EngineHandle initialize!");
    return 0;
}

int EngineHandle::hand_all(void)
{
    common_shm_process_handle();
    if(post_data.ip_get_flag == 1) //OLED
    {
        oled_ctrl.OLED_RowClear(0,1);
        oled_ctrl.OLED_ShowStr(0, 0, post_data.ip_address, 1);
        oled_ctrl.OLED_Screen_display();
        post_data.ip_get_flag = 0;
    }

    if(current_work_mode !=post_data.work_mode)
    {
        current_work_mode = post_data.work_mode;

        oled_ctrl.OLED_RowClear(1,1);
        if(current_work_mode==ASCBOT_OFF)
            oled_ctrl.OLED_ShowStr(1, 0, mode_off, 1);
        else if(current_work_mode == ASCBOT_VOID_DANGER_MODE)
            oled_ctrl.OLED_ShowStr(1, 0, mode_dange, 1);
        else if(current_work_mode == ASCBOT_OBJECT_MODE)
            oled_ctrl.OLED_ShowStr(1, 0, mode_follow, 1);
        else if(current_work_mode == ASCBOT_ROAD_MODE)
            oled_ctrl.OLED_ShowStr(1, 0, mode_road, 1);
        else if(current_work_mode == ASCBOT_REMOTE_MODE)
           oled_ctrl.OLED_ShowStr(1, 0, mode_remote, 1);

        oled_ctrl.OLED_Screen_display();

        current_wheel_left_speed = 0;
        current_wheel_right_speed = 0;
        wheel_ctrl.wheel_left_move(1,0x00);
        wheel_ctrl.wheel_right_move(1,0x00);

        shared_stuff->wheel_force_stop = 0;
    }

    INFO_LOG("current:%d,%d\n",current_wheel_left_speed,current_wheel_right_speed);
    if((current_wheel_left_speed!=post_data.wheel_left_speed)||(current_wheel_right_speed!=post_data.wheel_right_speed))
    {
        current_wheel_left_speed = post_data.wheel_left_speed;
        current_wheel_right_speed = post_data.wheel_right_speed;
        INFO_LOG(" current_wheel_left_speed:%d,current_wheel_right_speed:%d\n",current_wheel_left_speed,current_wheel_right_speed);

        if(current_wheel_left_speed<0)
            wheel_ctrl.wheel_left_move(-1,abs(current_wheel_left_speed));
        else
            wheel_ctrl.wheel_left_move(1,abs(current_wheel_left_speed));

        if(current_wheel_right_speed<0)
            wheel_ctrl.wheel_right_move(-1,abs(current_wheel_right_speed));
        else
            wheel_ctrl.wheel_right_move(1,abs(current_wheel_right_speed));
    }

    if((current_remote_left_speed != post_data.remote_left_speed) ||
    (current_remote_right_speed != post_data.remote_right_speed) ||
    (current_remote_direction != post_data.remote_direction))
    {
        current_remote_left_speed = post_data.remote_left_speed;
        current_remote_right_speed = post_data.remote_right_speed;
        current_remote_direction = post_data.remote_direction;
        wheel_ctrl.wheel_left_move(current_remote_direction,current_remote_left_speed);
        wheel_ctrl.wheel_right_move(current_remote_direction,current_remote_right_speed);
    }
    return 0;
}

int EngineHandle::HandleResults( ) {
    INFO_LOG("start Engine Handle Results!\n");
    hand_all();
    return 0;
}

int EngineHandle::HandleGetWorkMode() {
    return current_work_mode;
}

