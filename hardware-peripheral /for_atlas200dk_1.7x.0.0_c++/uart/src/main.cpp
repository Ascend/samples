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
#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "uart.h"

using namespace std;


char send_mid[] = "#0P1650T500\r";
char send_left[] = "#0P1180T500\r";
char send_right[] = "#0P2000T500\r";


int uart_test_axample(void)
{
    int ret;
    uart ctrl_uart;

    if(ctrl_uart.uart_open() < 0)
    {
        perror("uart_open error");
        return -1;
    }

    if((ret=ctrl_uart.uart_set_option(115200,8,'N',1)) < 0)
    {
        perror("uart_set_option error");
        return -1;
    }

    int cnt = 0;

    while (1)
    {
        if(cnt == 0)
            ctrl_uart.uart_send(send_left, sizeof(send_left));

        if(cnt == 1)
            ctrl_uart.uart_send(send_mid, sizeof(send_mid));

        if(cnt == 2)
            ctrl_uart.uart_send(send_right, sizeof(send_right));

        cnt ++;

        cnt%=3;

        usleep(3000000);
        printf("ctrl_uart.uart_send cnt %d!\n",cnt);



    }
    ctrl_uart.uart_close();
    return 0;

}


int main(int argc, char* argv[])
{
    //EXAMPLE
    uart_test_axample();
    //EXAMPLE end

}