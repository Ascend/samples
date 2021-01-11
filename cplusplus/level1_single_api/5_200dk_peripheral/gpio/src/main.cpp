/**
* @file Main.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <iostream>
#include <unistd.h>
#include "gpio.h"

using namespace std;

namespace{
    gpio io_ctrl;
}

int main(int argc, char* argv[])
{

    //EXAMPLE
    int cnt = 0;

    INFO_LOG("test gpio start\n");


    io_ctrl.gpio_set_direction(0,1);
    io_ctrl.gpio_set_direction(1,1);
    io_ctrl.gpio_set_direction(3,1);

    while(1)
    {

        INFO_LOG("GPIO TEST cnt=%d\n",cnt);

        /* red light */
        if(cnt==0)
        {
            io_ctrl.gpio_set_value(0,1);
            io_ctrl.gpio_set_value(1,0);
            io_ctrl.gpio_set_value(3,0);
        }

        /*green light */
        if(cnt==1)
        {
            io_ctrl.gpio_set_value(0,0);
            io_ctrl.gpio_set_value(1,1);
            io_ctrl.gpio_set_value(3,0);
        }

        /*blue light */
        if(cnt==2)
        {
            io_ctrl.gpio_set_value(0,0);
            io_ctrl.gpio_set_value(1,0);
            io_ctrl.gpio_set_value(3,1);
        }

        /*red green blue all  light */
        if(cnt==3)
        {
            io_ctrl.gpio_set_value(0,1);
            io_ctrl.gpio_set_value(1,1);
            io_ctrl.gpio_set_value(3,1);
        }


        /*red green blue  all  close */
        if(cnt==4)
        {
            io_ctrl.gpio_set_value(0,0);
            io_ctrl.gpio_set_value(1,0);
            io_ctrl.gpio_set_value(3,0);
        }

        cnt++;

        cnt%=5;

        usleep(3000000);
    }

    INFO_LOG("test gpio end\n");
    //end EXAMPLE
}