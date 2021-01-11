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

#include "uart.h"
#include<termios.h>    /*PPSIX*/

#define UART1_DEV_NAME                      "/dev/ttyAMA1"

uart::uart(void)
{

}

uart::~uart(void)
{

}


int uart::uart_open(void)
{
     fd = open(UART1_DEV_NAME,O_RDWR|O_NOCTTY|O_NDELAY);

     if (-1 == fd)
     {
         ERROR_LOG("Can't Open ttyAMA1");
         return(-1);
     }
     else
     {
         INFO_LOG("open ttyAMA1 success\n");
     }


     //recover uart block status to recevier datainput
    if(fcntl(fd, F_SETFL, 0) < 0)
        ERROR_LOG("fcntl failed!\n");
    else
        ERROR_LOG("fcntl=%d\n",fcntl(fd, F_SETFL,0));

    //check uart valid
    if(isatty(STDIN_FILENO)==0)
        ERROR_LOG("standard input is not a terminal device\n");
    else
        ERROR_LOG("isatty success!\n");

    INFO_LOG("fd-open=%d\n",fd);
    return fd;
}

int uart::uart_close(void)
{
    if(-1 == close(fd))
    {
        ERROR_LOG("uart close fd is wrong! ");
    }
    else
    {
        INFO_LOG("close ttyAMA1 success\n");
    }

}

int uart::uart_send(char *buffer,int size)
{
    return write(fd,buffer,size);
}

int uart::uart_read(char *buffer,int size)
{
    return read(fd,buffer,size);
}



int uart::uart_set_option(int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;

    //backup current uart setting
    if ( tcgetattr( fd,&oldtio) != 0)
    {
        ERROR_LOG("SetupSerial 1");
        return -1;
    }

    bzero( &newtio, sizeof( newtio ) );

    //set char size
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    //set data bit
    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    //set verify bit
    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    //set speed
    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    }

    //set stop bit
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;

    //set wait time and min receiver size
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    //clear receiver buffer
    tcflush(fd,TCIFLUSH);

    //reset setting
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        ERROR_LOG("com set error");
        return -1;
    }
    INFO_LOG("set uart new setting done!\n");
    return 0;
}


