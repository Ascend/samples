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

#include "wheel.h"


const unsigned char slave_addr = 0x60;

const unsigned char stop[70] = {0x01,0x04,0xe2,0xe4,0xe8,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,
                                0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00};



wheel::wheel(void)
{

}

wheel::~wheel(void)
{

}


int wheel::wheel_init(void)
{
    for(unsigned char i=0;i<70;i++)
    {
         if(i2c_ctrl.atlas_i2c_write(slave_addr,i,stop[i])!=0)
         {
            break;
         }
    }

    return 0;
}

int wheel::wheel_left_move(int direction,unsigned char speed)
{
    if(speed>0x0f)
        speed = 0x0f;

    INFO_LOG(" wheel left move\n");
    if(direction == 1)
    {
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2b,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2d,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2f,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x31,0x10);
    }
    else if(direction == -1)
    {
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2b,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2d,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x2f,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x31,0x00);
    }
    i2c_ctrl.atlas_i2c_write(slave_addr,0x29,speed);


    return 0;
}

int wheel::wheel_right_move(int direction,unsigned char speed)
{
    if(speed>0x0f)
        speed = 0x0f;

    if(direction == 1)
    {
        i2c_ctrl.atlas_i2c_write(slave_addr,0x33,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x35,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x37,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x39,0x00);
    }
    else if(direction == -1)
    {
        i2c_ctrl.atlas_i2c_write(slave_addr,0x33,0x10);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x35,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x37,0x00);
        i2c_ctrl.atlas_i2c_write(slave_addr,0x39,0x10);
    }
    i2c_ctrl.atlas_i2c_write(slave_addr,0x3d,speed);

    return 0;
}







