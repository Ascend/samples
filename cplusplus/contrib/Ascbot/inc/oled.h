#ifndef __OLED_H
#define __OLED_H

#include <linux/types.h>
#include <sys/ioctl.h>
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

#include <i2c.h>

#define OLED_ADDRESS 0x3C
#define OLED_Write_ADD (OLED_ADD << 1)
#define OLED_Read_ADD ((OLED_ADD << 1) + 1)
#define OLED_REG_CMD 0x00
#define OLED_REG_DAT 0x40

#define X_WIDTH 128
#define Y_WIDTH 32

#define OLED_CMD 0  //写命令
#define OLED_DATA 1 //写数据


class oled {
public:
    oled(void) ;
    ~oled(void) ;

    void Write_IIC_Command(uint8_t IIC_Command);
    void Write_IIC_Data(uint8_t IIC_Data);
    void OLED_WR_Byte(unsigned dat, unsigned cmd);
    //init
    void OLED_Init(void);   //only init one time
    void OLED_Fill(unsigned char fill_Data);// fill all screen ch
    void OLED_CLS(void);//clear screen
    void OLED_ON(void);//OLED on
    void OLED_OFF(void);//OLED off
    void OLED_CLS_RAM(void);  //clear screen ram,not clear screen
    void OLED_ShowStr(unsigned char row,unsigned char col, char *str, unsigned char TextSize);  // row  col show string
    void OLED_RowClear(unsigned char row, unsigned char TextSize);  //clear row ram data
    void OLED_SetDot(unsigned char x,unsigned char y,unsigned char color);   // draw x/y dot
    void OLED_Screen_display(void);   // show diplay ram data to screen
private:
    i2c i2c_ctrl;
    unsigned char OLED_GRAM[128][8];
};


#endif
