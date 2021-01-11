/**
* @file i2c.cpp
I2C ����ֻ��Ҫ�������ź�����ɣ� һ���Ǵ��������� SDA�� ��һ���Ǵ���ʱ���� SCL��
I2C ������˫��������ߣ���������ʹӻ������ܳ�Ϊ�������ͽ�������
���������ӻ��������ݣ��������Ƿ����������ӻ��ǽ���������������Ӵӻ���ȡ���ݣ��������ǽ����������ӻ��Ƿ�������
���������Ƿ��������ǽ�������ʱ���ź� SCL ��Ҫ������������
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

#include "i2c.h"

int i2c::i2c_write(unsigned char slave, unsigned char reg, unsigned char value)
{
    int ret;
    struct i2c_rdwr_ioctl_data ssm_msg = {0};
    unsigned char buf[2] = {0};
    ssm_msg.nmsgs = 1;
    ssm_msg.msgs = (struct i2c_msg *)malloc(ssm_msg.nmsgs * sizeof(struct i2c_msg));
    if (ssm_msg.msgs == NULL) {
        ERROR_LOG("Memory alloc error!\n");
        return -1;
    }
    buf[0] = reg;
    buf[1] = value;
    (ssm_msg.msgs[0]).flags = 0;
    (ssm_msg.msgs[0]).addr = (unsigned short)slave;
    (ssm_msg.msgs[0]).buf = buf;
    (ssm_msg.msgs[0]).len = 2;
    ret = ioctl(fd, I2C_RDWR, &ssm_msg);
    if (ret < 0) {
        ERROR_LOG("write error, ret=%#x, errorno=%#x, %s!\n", ret, errno, strerror(errno));
        free(ssm_msg.msgs);
        ssm_msg.msgs = NULL;
        return -1;
    }
    free(ssm_msg.msgs);
    ssm_msg.msgs = NULL;
    return 0;
}

int i2c::i2c_read(unsigned char slave, unsigned char reg, unsigned char *buf)
{
    int ret;
    struct i2c_rdwr_ioctl_data ssm_msg = {0};
    unsigned char regs[2] = {0};
    regs[0] = reg;
    regs[1] = reg;
    ssm_msg.nmsgs = 2;
    ssm_msg.msgs = (struct i2c_msg *)malloc(ssm_msg.nmsgs * sizeof(struct i2c_msg));
    if (ssm_msg.msgs == NULL) {
        ERROR_LOG("Memory alloc error!\n");
        return -1;
    }
    (ssm_msg.msgs[0]).flags = 0;
    (ssm_msg.msgs[0]).addr = slave;
    (ssm_msg.msgs[0]).buf = regs;
    (ssm_msg.msgs[0]).len = 1;
    (ssm_msg.msgs[1]).flags = I2C_M_RD;
    (ssm_msg.msgs[1]).addr = slave;
    (ssm_msg.msgs[1]).buf = buf;
    (ssm_msg.msgs[1]).len = 1;
    ret = ioctl(fd, I2C_RDWR, &ssm_msg);
    if (ret < 0) {
        ERROR_LOG("read data error,ret=%#x !\n", ret);
        free(ssm_msg.msgs);
        ssm_msg.msgs = NULL;
        return -1;
    }
    free(ssm_msg.msgs);
    ssm_msg.msgs = NULL;
    return 0;
}
/*
 * i2c_init, for access i2c device.
 */
int i2c::i2c_2_init()
{
    // open i2c-2 device
    fd = open(I2C2_DEV_NAME, O_RDWR);
    if (fd < 0) {
        ERROR_LOG("i2c-2 Can't open !\n");
        return -1;
    }
    // set i2c-1 retries time
    if (ioctl(fd, I2C_RETRIES, 1) < 0) {
        close(fd);
        fd = 0;
        ERROR_LOG("set i2c-2 retry fail!\n");
        return -1;
    }
    // set i2c-1 timeout time, 10ms as unit
    if (ioctl(fd, I2C_TIMEOUT, 1) < 0) {
        close(fd);
        fd = 0;
        ERROR_LOG("set i2c-2 timeout fail!\n");
        return -1;
    }
    return 0;
}

i2c::i2c(void)
{
    i2c_2_init();
}

i2c::~i2c(void)
{

}

int i2c::atlas_i2c_get_fd(void)
{
    return fd;
}


int i2c::atlas_i2c_write(unsigned char slave, unsigned char reg, unsigned char data)
{
    int ret;
    ret = i2c_write(slave, reg, data);
    if (ret != 0) {
        close(fd);
        fd = 0;
        ERROR_LOG("atlas_i2c_write %#x %#x to %#x fail!\n", slave, data, reg);
        return -1;
    }
    return 0;
}


int i2c::atlas_i2c_read(unsigned char slave, unsigned char reg, unsigned char *data)
{
    int ret;
    ret = i2c_read(slave, reg, data);
    if (ret != 0) {
         close(fd);
         fd = 0;
         ERROR_LOG("atlas_i2c_read  fail!\n");
         return -1;
    }
    return 0;
}

int i2c::i2c_smbus_access(int file, char read_write,unsigned char command,int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

	int ret = ioctl(fd,I2C_SMBUS,&args);
	return ret;
}

int i2c::atlas_setI2CSlave(int slave)  //for smbus mode
{
	if (ioctl(fd, I2C_SLAVE, slave) < 0)
	{
        ERROR_LOG("Fail ioctl I2C_SLAVE");
		return -1;
	}
	return 0;
}


int i2c::atlas_i2c_smbus_write_quick(unsigned char value)
{
	return i2c_smbus_access(fd,value,0,I2C_SMBUS_QUICK,NULL);
}

int i2c::atlas_i2c_smbus_read_byte(void)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(fd,I2C_SMBUS_READ,0,I2C_SMBUS_BYTE,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int i2c::atlas_i2c_smbus_write_byte(unsigned char value)
{
	return i2c_smbus_access(fd,I2C_SMBUS_WRITE,value,I2C_SMBUS_BYTE,NULL);
}

int i2c::atlas_i2c_smbus_read_byte_data(unsigned char command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(fd,I2C_SMBUS_READ,command,I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int i2c::atlas_i2c_smbus_write_byte_data(unsigned char command,unsigned char value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_BYTE_DATA, &data);
}

int i2c::atlas_i2c_smbus_read_word_data(unsigned char command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(fd,I2C_SMBUS_READ,command,I2C_SMBUS_WORD_DATA,&data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

int i2c::atlas_i2c_smbus_write_word_data(unsigned char command,unsigned short value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_WORD_DATA, &data);
}

int i2c::atlas_i2c_smbus_process_call(unsigned char command, unsigned short value)
{
	union i2c_smbus_data data;
	data.word = value;
	if (i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_PROC_CALL,&data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

/* Returns the number of read bytes */
int i2c::atlas_i2c_smbus_read_block_data(unsigned char command,unsigned char *values)
{
	union i2c_smbus_data data;
	int i;
	if (i2c_smbus_access(fd,I2C_SMBUS_READ,command,I2C_SMBUS_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}

int i2c::atlas_i2c_smbus_write_block_data(unsigned char command,unsigned char length, unsigned char *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > 32)
		length = 32;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_BLOCK_DATA, &data);
}

/* Returns the number of read bytes */
int i2c::atlas_i2c_smbus_read_i2c_block_data(unsigned char command,unsigned char *values)
{
	union i2c_smbus_data data;
	int i;
	if (i2c_smbus_access(fd,I2C_SMBUS_READ,command,I2C_SMBUS_I2C_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}

int i2c::atlas_i2c_smbus_write_i2c_block_data(unsigned char command,unsigned char length, unsigned char *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > 32)
		length = 32;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

/* Returns the number of read bytes */
int i2c::atlas_i2c_smbus_block_process_call(unsigned char command,unsigned char length, unsigned char *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > 32)
		length = 32;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	if (i2c_smbus_access(fd,I2C_SMBUS_WRITE,command,I2C_SMBUS_BLOCK_PROC_CALL,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}





