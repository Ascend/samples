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
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>
#include <signal.h>
#include<math.h>
#include <sys/shm.h>

#include <net/if.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>



#include "common_post.h"

#include "uart_print.h"
#include "tcp_server.h"
using namespace std;

#define PORT  55566
#define QUEUE_SIZE   10
#define BUFFER_SIZE 1024

namespace{
uart_print UART_PRINT;

char local_ip_addr[20];

struct shared_common_st *shared_stuff;
void *shared_memory = (void *)0;
int shmid;
}


tcp_server::tcp_server(void)
{

}
tcp_server::~tcp_server(void)
{

}


int tcp_server::server_shm_init(void)
{
    UART_PRINT.UPRINT("camera_shm_int------------ \n");
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
    shared_stuff->wheel_speed_written_flag = 0;
    shared_stuff->ip_written_flag = 0 ;
    shared_stuff->image_data_written_flag = 0;
    shared_stuff->image_send_status = 0;

    return 0;
}


int process_recevier_data(unsigned char cmd,unsigned char param1,unsigned char param2,unsigned char param3)
{
    switch (cmd)
    {
        case 1:  //red
            {
            }
            break;
        case 2:  //green
            {
            }
           break;
        case 3:  //yellow
            {
            }
            break;
        case 4: // blue
            {
            }
            break;
        case 5:  // control joystick
            {
                int x = (int)param2-100;
                int y = (int)param3-100;

                int left;
                int right;
                int direction;

                if(y<0)
                {
                    left =  fabs((y*15)/100);
                    right = fabs((y*15)/100);
                    direction = -1;
                }
                else
                {
                    if(x>0)
                    {
                        left =  fabs((y*15)/100)+fabs((x*15)/100)/4;
                        right = fabs((y*15)/100)-fabs((x*15)/100)/4;
                        if(left>15)
                            left = 15;
                        if(right<0)
                            right = 0;
                    }
                    else
                    {
                        left =  fabs((y*15)/100)-fabs((x*15)/100)/4;
                        right = fabs((y*15)/100)+fabs((x*15)/100)/4;
                        if(right>15)
                               right = 15;
                        if(left<0)
                               left = 0;
                    }


                    direction = 1;

                }
                //UART_PRINT.UPRINT("set speed.........%d  %d %d \n",direction,left,right);
                if(shared_stuff->written_flag==0)
                {
                    shared_stuff->remote_direction = direction;
                    shared_stuff->remote_left_speed = left;
                    shared_stuff->remote_right_speed = right;
                    shared_stuff->written_flag = 1;
                }
            }
            break;
        case 6: // round switch
            {
                if(shared_stuff->written_flag==0)
                {
                    shared_stuff->work_mode = param1;
                    shared_stuff->remote_direction = 0;
                    shared_stuff->remote_left_speed = 0;
                    shared_stuff->remote_right_speed = 0;
                    shared_stuff->written_flag = 1;
                }
            }
            break;
        case 7: // video switch
            {
                if(shared_stuff->written_flag==0)
                {
                    shared_stuff->video_flag = param1;
                    shared_stuff->image_send_status = param1;
                    shared_stuff->written_flag = 1;
                }
            }
            break;
        default:
            break;
    }
}

void send_image_echo(int sockfd)
{
    unsigned char buffer[BUFFER_SIZE];
    pid_t pid = getpid();

    unsigned char head[8];
    int status;
    while(1)
    {
        status = shared_stuff->image_send_status;
        if(status == 0)
        {
            usleep(500*1000);
            UART_PRINT.UPRINT("send_image_echo ..-------none----------- sockfd = %d  \n",sockfd);
        }
        else if(status == 101)
        {
            UART_PRINT.UPRINT("send_image_echo ..-------exit----------- sockfd = %d  \n",sockfd);
            break;
        }
        else
        {
            usleep(10*1000);
            if (shared_stuff->image_data_written_flag==1)
            {
                 int nCount = shared_stuff->image_data_size;
                 int nret;

                 head[0]= 100;
                 head[1]= shared_stuff->image_data_width;
                 head[2]= shared_stuff->image_data_height;
                 head[3]= 0;
                 head[4]= (nCount>>24)&0xff;
                 head[5]= (nCount>>16)&0xff;
                 head[6]= (nCount>>8)&0xff;
                 head[7]= (nCount)&0xff;

                 nret = send(sockfd,head, 8, 0);

                 if(nret == -1)
                 {
                    UART_PRINT.UPRINT("send_image_echo .error---------------------------- %d  size = %d  fd=%d\n",nret,nCount,sockfd);
                    shared_stuff->image_send_status = 0;
                    break;
                 }

                 nret = send(sockfd,shared_stuff->image_data, nCount, 0);
                 UART_PRINT.UPRINT("send_image_echo ..---------------------------- %d  size = %d  fd=%d\n",nret,nCount,sockfd);
                 shared_stuff->image_data_written_flag = 0;
            }
        }
    }
    shared_stuff->image_send_status = 0;
}

void str_echo(int sockfd)
{
    char buffer[BUFFER_SIZE];
    pid_t pid = getpid();

    while(1)
    {
        usleep(100*1000);
        memset(buffer,0,sizeof(buffer));
        int len = recv(sockfd, buffer, sizeof(buffer),0);
        if(strcmp(buffer,"exit123")==0)
        {
            ERROR_LOG("child process: %d exited.\n",pid);
            shared_stuff->image_send_status = 101;
            usleep(500*1000);
            break;
        }
        INFO_LOG("received cmd = %d %d %d %d %d %d \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
        if((buffer[0]==0)&&(buffer[1]==0)&&(buffer[2]==0)&&(buffer[4]==0))
        {
              INFO_LOG("child process: %d exited. not normal\n",pid);
              break;
        }

        if((buffer[0]==0xfa)&&(buffer[1]==0xaa))
        {
            process_recevier_data(buffer[2],buffer[3],buffer[4],buffer[5]);
        }
    }
    close(sockfd);
}



int get_localip(const char * eth_name, char *local_ip_addr)
{
	int ret = -1;
    register int fd;
    struct ifreq ifr;

	if (local_ip_addr == NULL || eth_name == NULL)
	{
		return ret;
	}
	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) > 0)
	{
		strcpy(ifr.ifr_name, eth_name);
		if (!(ioctl(fd, SIOCGIFADDR, &ifr)))
		{
			ret = 0;
			strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
		}
	}
	if (fd > 0)
	{
		close(fd);
	}
    return ret;

}

void tcp_server_process(int a,int b)
{
    int reuse = 0;
    int server_sockfd = socket(AF_INET,SOCK_STREAM, 0);

    while(1)
    {
        if( get_localip("eth0",local_ip_addr)==0)
        {
            if(shared_stuff->ip_written_flag==0)
            {
                strncpy(shared_stuff->ip_address ,local_ip_addr, 20);
                shared_stuff->ip_written_flag = 1;
            }
            UART_PRINT.UPRINT("get_ip_addr success = %s\n",local_ip_addr);
            break;
        }
        else
        {
            UART_PRINT.UPRINT("get_ip_addr error= %s\n",local_ip_addr);
            usleep(2000000);
        }

    }
    struct sockaddr_in server_sockaddr;
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_sockaddr.sin_port = htons(PORT);

    if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        UART_PRINT.UPRINT("setsockopet error\n");
        perror("setsockopet error\n");
        exit(1);
    }

    while(1)
    {
        if(bind(server_sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))==-1)
        {
            UART_PRINT.UPRINT("bind error\n");
            perror("bind");
            usleep(5000000);
        }
        else
            break;
    }

    UART_PRINT.UPRINT("bind success.\n");
    INFO_LOG("bind success.\n");

    if(listen(server_sockfd,QUEUE_SIZE) == -1)
    {
        INFO_LOG("listen");
        exit(1);
    }
    INFO_LOG("listen success.\n");

    for(;;)
    {
        usleep(100*1000);
        struct sockaddr_in client_addr;
        socklen_t length = sizeof(client_addr);

        int conn = accept(server_sockfd, (struct sockaddr*)&client_addr,&length);
        if(conn<0)
        {
            ERROR_LOG("connect");
            exit(1);
        }
        INFO_LOG("new client accepted.\n");
        pid_t childid;

        if(childid=fork()==0)//child process
        {
            INFO_LOG("send image child process: %d created.\n", getpid());
            send_image_echo(conn);
            INFO_LOG("send image child process send_image_echo quit!\n");
            exit(0);
        }

        if(childid=fork()==0)//child process
        {
            UART_PRINT.UPRINT("child process: %d created.\n", getpid());
            close(server_sockfd);
            str_echo(conn);
            UART_PRINT.UPRINT("child process str_echo quit!\n");
            exit(0);
        }
    }

    INFO_LOG("closed.\n");
    close(server_sockfd);
}

int tcp_server::tcp_server_create(void)
{
    server_shm_init();
    INFO_LOG("thread tcp_server_create\n");
    thread thread_server_process(&tcp_server_process, 0, 0);
    thread_server_process.detach();
    return 0;
}
 

