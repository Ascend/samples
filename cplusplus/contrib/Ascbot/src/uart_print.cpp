/**
* @file uart.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
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
#include "uart_print.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <iostream>
#include<termios.h>

#define UART0_DEV_NAME                      "/dev/ttyAMA0"
uart_print::uart_print(void)
{
    #if UART_PRINT_ENABLE
    uart_print_init();
    #endif
}

uart_print::~uart_print(void)
{

}
int uart_print::uart_print_init(void)
{
    // open uart device
    fd = open(UART0_DEV_NAME, O_RDWR);  //|O_NONBLOCK|O_NOCTTY|O_NDELAY
    if (fd < 0) {
        ERROR_LOG("UART Can't open uart0 print!");
        return -1;
    }

    return 0;
}

int uart_print::uart_print_close(void)
{
    close(fd);
    return 0;
}

int uart_print::uart_output_char(void)
{
     int len, i,ret;
     len = write(fd, uart_info, uart_info_index);
     memset(uart_info,0,sizeof(uart_info));
     return len;
}

void uart_print::putccc(char a)
{
    char buf[1];
    buf[0] = a;
    if(uart_info_index >=999)
        return;

    uart_info[uart_info_index] = buf[0];
    uart_info_index++;
}

void uart_print::putInt(int n){
    if(n<0)
    {
        putccc('-');
        n = abs(n);
    }
    if(n>9){
        putInt(n/10);
    }
    putccc(n%10+ '0');
}

void uart_print::putHex(int n){

    if(n<0)
    {
        putccc('-');
        n = abs(n);
    }

    if(n>15){
        putHex(n/16);
    }
    if(n%16<10)
        putccc(n%16+ '0');
    else
        putccc(n%16-10+ 'A');

}


void uart_print::putfloat(double n){

    if(n<0)
    {
        putccc('-');
        n = abs(n);
    }

    double tmp = n*1000;

    int k1 = (int)tmp;

    putInt(k1/1000);
    putccc('.');
    putInt(k1%1000);
}

int uart_print::UPRINT(const char *format, ...)
{
#if UART_PRINT_ENABLE
    uart_info_index = 0 ;
    assert(format);
    va_list arg;
    va_start(arg, format);
    const char *start=format;
    while (*start!= '\0')
    {
        if(*start =='%'){
            start++;
            switch(*start)
            {
                case 'd':
                    putInt(va_arg(arg, int));
                    break;
                case 'x':
                case 'X':
                    putHex(va_arg(arg, int));
                    break;
                case 'c':
                    putccc(va_arg(arg, int));
                    break;
                case 's':
                    {
                        char *ch = va_arg(arg, char*);
                        while (*ch)
                        {
                            putccc(*ch);
                            ch++;
                        }
                    }
                    break;
                case 'f':
                    putfloat(va_arg(arg, double));
                    break;
                default :
                    break;
            }
        }
        else
        {
            putccc(*start) ;
        }
        start++;
    }
    va_end(arg);
    putccc('\0');
    uart_output_char();
#endif
    return 0;
}
