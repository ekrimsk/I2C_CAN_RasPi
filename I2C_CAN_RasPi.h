#ifndef __I2C_CAN_RASPI_H__
#define __I2C_CAN_RASPI_H__



#include <stdint.h> // for uint
#define byte uint8_t // so we dont need to edit all the instances
#include <wiringPiI2C.h> // rewriting for the Pi 
#include <wiringPi.h> // for micros, remmvoe if dont need



#define MIN_WRITE_DELAY 100 // 100 works, can potentially go lower than 50 too

#include "I2C_CAN_dfs.h"

#include <unistd.h>  // usleep

#include <stdio.h>  // for debug



#define MCP_CAN I2C_CAN

class I2C_CAN{

private:

    unsigned char IIC_ADDR;
    unsigned long m_ID;
    unsigned char m_RTR;
    unsigned char m_EXT;
    
    unsigned char makeCheckSum(unsigned char *dta, int len);

    bool clear_flag = false;

    // Added for RasPi
    int _fd = 0x00; // file descriptor, will get edited in initialization
    
public:
    I2C_CAN(unsigned char __addr);
    ~I2C_CAN();


    void begin();   
    void IIC_CAN_SetReg(unsigned char __reg, unsigned char __len, unsigned char *__dta);
    void IIC_CAN_SetReg(unsigned char __reg, unsigned char __dta);
    bool IIC_CAN_GetReg(unsigned char __reg, unsigned char *__dta);
    bool IIC_CAN_GetReg(unsigned char __reg, int len, unsigned char *__dta);
    void clear_buffer(void);



    byte checkReceive(void);                                        // if something received
    int framesAvail(void);
    
    byte begin(byte speedset);                                      // init can 
    byte init_Mask(byte num, byte ext, unsigned long ulData);       // init Masks
    byte init_Filt(byte num, byte ext, unsigned long ulData);       // init filters
    byte sendMsgBuf(unsigned long id, byte ext, byte rtr, byte len, byte *buf);     // send buf
    byte sendMsgBuf(unsigned long id, byte ext, byte len, byte *buf);               // send buf
    byte readMsgBuf(byte *len, byte *buf);                          // read buf
    byte readMsgBufID(unsigned long *ID, byte *len, byte *buf);     // read buf with object ID
    byte checkError(void);                                          // if something error
    unsigned long getCanId(void);                                   // get can id when receive
    byte isRemoteRequest(void);                                     // get RR flag when receive
    byte isExtendedFrame(void);                                     // did we recieve 29bit frame?

    
};

#endif