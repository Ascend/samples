/**
* @file i2c.h
*/

#ifndef I2C_H_
#define I2C_H_

#include <sys/types.h>
#include "utils.h"


/* I2C Device*/
#define I2C2_DEV_NAME                      "/dev/i2c-2"

#define I2C_M_TEN	0x10	/* we have a ten bit chip address	*/
#define I2C_M_RD	0x01
#define I2C_M_NOSTART	0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

#define I2C_RETRIES	0x0701	/* number of times a device address      */
#define I2C_TIMEOUT	0x0702	/* set timeout - call with int 		*/
/* this is for i2c-dev.c	*/
#define I2C_SLAVE	0x0703	/* Change slave address			*/
#define I2C_SLAVE_FORCE	0x0706	/* Change slave address			*/
#define I2C_TENBIT	0x0704	/* 0 for 7 bit addrs, != 0 for 10 bit	*/
#define I2C_FUNCS	0x0705	/* Get the adapter functionality */
#define I2C_RDWR	0x0707	/* Combined R/W transfer (one stop only)*/
#define I2C_PEC		0x0708	/* != 0 for SMBus PEC                   */
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */


/*
 * Data for SMBus Messages
 */


/* smbus_access read or write markers */
#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_DATA    6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_BLOCK_DATA_PEC    8		/* SMBus 2.0 */
#define I2C_SMBUS_PROC_CALL_PEC     9		/* SMBus 2.0 */
#define I2C_SMBUS_BLOCK_PROC_CALL_PEC  10	/* SMBus 2.0 */
#define I2C_SMBUS_WORD_DATA_PEC	   11		/* SMBus 2.0 */


#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */

class i2c {

public:
    i2c(void) ;
    ~i2c(void) ;

    //normal
    int atlas_i2c_get_fd(void);
    int atlas_i2c_write(unsigned char slave, unsigned char reg, unsigned char data);
    int atlas_i2c_read(unsigned char slave, unsigned char reg, unsigned char *data);

    //smbus
    int atlas_setI2CSlave(int slave);
    int atlas_i2c_smbus_write_quick(unsigned char value);
    int atlas_i2c_smbus_read_byte(void);
    int atlas_i2c_smbus_write_byte(unsigned char value);
    int atlas_i2c_smbus_read_byte_data(unsigned char command);
    int atlas_i2c_smbus_write_byte_data(unsigned char command,unsigned char value);
    int atlas_i2c_smbus_read_word_data( unsigned char command);
    int atlas_i2c_smbus_write_word_data(unsigned char command,unsigned short value);
    int atlas_i2c_smbus_process_call(unsigned char command, unsigned short value);
    int atlas_i2c_smbus_read_block_data(unsigned char command,unsigned char *values);
    int atlas_i2c_smbus_write_block_data(unsigned char command,unsigned char length, unsigned char *values);
    int atlas_i2c_smbus_read_i2c_block_data(unsigned char command,unsigned char *values);
    int atlas_i2c_smbus_write_i2c_block_data(unsigned char command,unsigned char length, unsigned char *values);
    int atlas_i2c_smbus_block_process_call(unsigned char command,unsigned char length, unsigned char *values);

private:
    int fd;
    struct i2c_msg {
        unsigned short addr;     /* slave address */
        unsigned short flags;
        unsigned short len;
        unsigned char *buf;     /* message data pointer */
    };
    struct i2c_rdwr_ioctl_data {
        struct i2c_msg *msgs;   /* i2c_msg[] pointer */
        int nmsgs;              /* i2c_msg Nums */
    };

    union i2c_smbus_data {
        unsigned char byte;
        unsigned short word;
        unsigned char block[I2C_SMBUS_BLOCK_MAX + 3]; /* block[0] is used for length */
                              /* one more for read length in block process call */
                                                    /* and one more for PEC */
    };
    struct i2c_smbus_ioctl_data {
        char read_write;
        unsigned char command;
        int size;
        union i2c_smbus_data *data;
    };

private:
    int i2c_write(unsigned char slave, unsigned char reg, unsigned char value);
    int i2c_read(unsigned char slave, unsigned char reg, unsigned char *buf);
    int i2c_2_init();

    int i2c_smbus_access(int file, char read_write,unsigned char command,int size, union i2c_smbus_data *data);

};

#endif // I2C_H_
