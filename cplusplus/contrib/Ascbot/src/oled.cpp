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

#include <linux/ip.h>
#include <linux/fs.h>

#include "oled.h"
#include "i2c.h"
#include "oled_data.h"



oled::oled(void)
{
}

oled::~oled(void)
{

}


/**********************************************
// IIC Write Command
**********************************************/
void oled::Write_IIC_Command(uint8_t IIC_Command)
{
	i2c_ctrl.atlas_i2c_smbus_write_byte_data(OLED_REG_CMD,IIC_Command);
}

/**********************************************
// IIC Write Data
**********************************************/
void oled::Write_IIC_Data(uint8_t IIC_Data)
{
    i2c_ctrl.atlas_i2c_smbus_write_byte_data( OLED_REG_DAT, IIC_Data);
}

/**********************************************
// IIC Write byte
**********************************************/
void oled::OLED_WR_Byte(unsigned dat, unsigned cmd)
{
	if (cmd){
		Write_IIC_Data(dat);
	}
	else
	{
		Write_IIC_Command(dat);
	}
}

//OLED init
void oled::OLED_Init(void)
{
    unsigned char  m,n;
    if(i2c_ctrl.atlas_setI2CSlave(OLED_ADDRESS) < 0)
    {
        ERROR_LOG("SET SLAVE failed...\r\n");
    }
	usleep(200*1000);

    for(m=0;m<128;m++)
        for(n=0;n<4;n++)
            OLED_GRAM[m][n]=0;

    OLED_WR_Byte(0xAE,OLED_CMD);//close display

    OLED_WR_Byte(0x40,OLED_CMD);//---set low column address
    OLED_WR_Byte(0xB0,OLED_CMD);//---set high column address

    OLED_WR_Byte(0xC8,OLED_CMD);//-not offset

    OLED_WR_Byte(0x81,OLED_CMD);//
    OLED_WR_Byte(0xff,OLED_CMD);

    OLED_WR_Byte(0xa1,OLED_CMD);//

    OLED_WR_Byte(0xa6,OLED_CMD);//

    OLED_WR_Byte(0xa8,OLED_CMD);//set driver number
    OLED_WR_Byte(0x1f,OLED_CMD);

    OLED_WR_Byte(0xd3,OLED_CMD);
    OLED_WR_Byte(0x00,OLED_CMD);

    OLED_WR_Byte(0xd5,OLED_CMD);
    OLED_WR_Byte(0xf0,OLED_CMD);

    OLED_WR_Byte(0xd9,OLED_CMD);
    OLED_WR_Byte(0x22,OLED_CMD);

    OLED_WR_Byte(0xda,OLED_CMD);
    OLED_WR_Byte(0x02,OLED_CMD);

    OLED_WR_Byte(0xdb,OLED_CMD);
    OLED_WR_Byte(0x49,OLED_CMD);

    OLED_WR_Byte(0x8d,OLED_CMD);
    OLED_WR_Byte(0x14,OLED_CMD);

    OLED_WR_Byte(0xaf,OLED_CMD);
}



void oled::OLED_Fill(unsigned char fill_Data)//fill 
{
    unsigned char m, n;
    for(m = 0; m < 4; m++){
        OLED_WR_Byte(0xb0 + m, OLED_CMD);	//page0-page1
        OLED_WR_Byte(0x00, OLED_CMD);		//low column start address
        OLED_WR_Byte(0x10, OLED_CMD);		//high column start address
        for(n=0;n<128;n++){
            OLED_WR_Byte(fill_Data, OLED_DATA);
        }
    }
}

void oled::OLED_CLS(void)
{
    OLED_Fill(0x00);
}

void oled::OLED_ON(void)//OLED on
{
    OLED_WR_Byte(0X8D, OLED_CMD);  
    OLED_WR_Byte(0X14, OLED_CMD); 
    OLED_WR_Byte(0XAF, OLED_CMD);  //OLED awake
}

void oled::OLED_OFF(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);  
    OLED_WR_Byte(0X10, OLED_CMD);  
    OLED_WR_Byte(0XAE, OLED_CMD);  //OLED close、
}

/**************************************/
/**************************************/
// draw point
//x,y  screen display pos   x 0-127   y 0-31
//color  :color 0：not show  1：show
/**************************************/
void oled::OLED_SetDot(unsigned char x,unsigned char y,unsigned char color)
{
    unsigned char  pos ,bx,temp=0;
    if(x>127||y>31) return;
    pos=y/8;
    bx=y%8;
    temp=1<<(bx);
    if(color==1)
        OLED_GRAM[x][pos]|=temp;
    else
        OLED_GRAM[x][pos]&=~temp;
}

/**************************************/
//Display all panel data
/**************************************/
void oled::OLED_Screen_display(void)
{
    unsigned char  m=0,n=0;

    for (m=0;m<4;m++){
        OLED_WR_Byte(0xb0+m, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for(n=0;n<128;n++)
        {
            OLED_WR_Byte(OLED_GRAM[n][m], OLED_DATA);
        }

    }
}

/**************************************/
// clear ram data
/**************************************/
void oled::OLED_CLS_RAM(void)
{
    unsigned char  m,n;
    for(m=0;m<128;m++)
        for(n=0;n<4;n++)
            OLED_GRAM[m][n]=0;
}

//--------------------------------------------------------------
//void oled::OLED_ShowStr(unsigned char row,unsigned char col, unsigned char *str, unsigned char TextSize)
// Parameters     : row  (0~32/font_height )  col (0~128/font_width)  str (show string)
// TextSize -- (1:6*8 ; 2:8*16)
// Description    :oled_data.h  ASCII  6*8   8*16
//--------------------------------------------------------------
void oled::OLED_ShowStr(unsigned char row,unsigned char col, char *str, unsigned char TextSize)
{
    unsigned char c = 0, i = 0;

    switch(TextSize)
    {
        case 1:  // 6 * 8
        {
            while(*str != '\0'){
                c = *str - 32;

                if(row>=32/8) break;
                if(col>=128/6) break;

                for(i = 0; i < 6; i++)
                    OLED_GRAM[6*col + i][row] |= (F6x8[c][i]);
                col++;
                str++;
            }
        }
        break;

        case 2:  //8 * 16
        {

            while(*str != '\0'){
                c = *str - 32;

                if(row>=32/16) break;
                if(col>=128/8) break;

                for(i=0;i<8;i++){
                    OLED_GRAM[8*col+i][2*row]|=(F8X16[c*16+i]);
                }
                for(i=0;i<8;i++){
                    OLED_GRAM[8*col+i][2*row+1]|=(F8X16[c*16+i+8]);
                }
                col++;
                (*str)++;
            }
        }
        break;

        default:
            break;
    }
}

void oled::OLED_RowClear(unsigned char row, unsigned char TextSize)
{
    unsigned char  m,n;
    switch(TextSize)
    {
        case 1:  // 6*8
        {
            if(row>=32/8) break;
            for(m=0;m<128;m++)
                OLED_GRAM[m][row]=0;

        }
        break;
        case 2: //8*16
        {
             if(row>=32/16) break;
             for(m=0;m<128;m++)
             {
                 OLED_GRAM[m][2*row]=0;
                 OLED_GRAM[m][2*row+1]=0;
             }
        }
        break;
        default:
            break;
    }
}
