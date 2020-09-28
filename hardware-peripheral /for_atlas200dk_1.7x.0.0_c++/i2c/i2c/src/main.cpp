/**
* @file Main.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <iostream>
#include <unistd.h>
#include "i2c.h"

using namespace std;

int main(int argc, char* argv[])
{
    //EXAMPLE
    i2c i2c_ctrl;

    const unsigned char slave_addr = 0x40;

    const unsigned char stop[70] = {0x00,0x04,0xe2,0xe4,0xe8,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,
    0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x00,0x00,
    0x00,0x10,0x00,0x00,0x00,0x10};


    for(unsigned char i=0;i<70;i++)
    {
        if(i2c_ctrl.atlas_i2c_write(slave_addr,i,stop[i])!=0)
        {
            break;
        }
    }


    int cnt = 0;


    unsigned short value = 5*160;;
    while(1)
    {
        cnt++;

        if(cnt%2==0)
        {
            i2c_ctrl.atlas_i2c_write(slave_addr,0x0c,0x00);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x0d,0x00);

            i2c_ctrl.atlas_i2c_write(slave_addr,0x10,value&0xff);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x11,value>>8);

            i2c_ctrl.atlas_i2c_write(slave_addr,0x08,0xa0);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x09,0x0f);



        }
        else
        {
            i2c_ctrl.atlas_i2c_write(slave_addr,0x0c,value&0xff);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x0d,value>>8);

            i2c_ctrl.atlas_i2c_write(slave_addr,0x10,0x00);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x11,0x00);

            i2c_ctrl.atlas_i2c_write(slave_addr,0x08,0xa0);
            i2c_ctrl.atlas_i2c_write(slave_addr,0x09,0x0f);

        }

        usleep(3000000);

    }


    //end EXAMPLE

}