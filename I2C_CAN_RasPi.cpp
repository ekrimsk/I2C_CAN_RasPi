#include "I2C_CAN_RasPi.h"




// See https://docs.longan-labs.cc/1030017/ for DOCs

I2C_CAN::I2C_CAN(unsigned char __addr)
{
    IIC_ADDR = __addr;


    // Remove if only use for micros
    //wiringPiSetupSys();  // needed for timing 


    // For default I2c
    //_fd =  wiringPiI2CSetup(IIC_ADDR); // get file descriptor 
    // For I2C 5 which we have configured separately 
    //_fd =  wiringPiI2CSetupInterface("/dev/i2c-5", IIC_ADDR); // get file descriptor 
    _fd =  wiringPiI2CSetupInterface("/dev/i2c-0", IIC_ADDR); // get file descriptor 

}

I2C_CAN::~I2C_CAN(){
    clear_buffer();
}

void I2C_CAN::begin()
{
    // DO Nothing -- default speed 
    clear_buffer();
}

byte I2C_CAN::begin(byte speedset)                                      // init can
{
    /*
    Wire.begin();
    
    IIC_CAN_SetReg(REG_BAUD, speedset);
    delay(10);
    
    unsigned char __speed = 0;
    
    if(IIC_CAN_GetReg(REG_BAUD, &__speed))
    {
        if(speedset == __speed)return 1;
        
    }
    
    delay(100);

    return 0;
    */


    


    printf("Setting I2C-CAN motor baud...\n");
    
    IIC_CAN_SetReg(REG_BAUD, speedset);
    //delay(10);
    I2C_sleep(10000);
    unsigned char __speed = 0;
    
    if(IIC_CAN_GetReg(REG_BAUD, &__speed))
    {
        if(speedset == __speed)return 1;
        
    }
    
    //delay(100);
    //I2C_sleep(100000);
    I2C_sleep(90000);



    clear_buffer();


    return 0;
}


void I2C_CAN::IIC_CAN_SetReg(unsigned char __reg, unsigned char __len, unsigned char *__dta)
{
    /*
    Wire.beginTransmission(IIC_ADDR);
    Wire.write(__reg);
    for(int i=0; i<__len; i++)
    {
        Wire.write(__dta[i]);
    }
    Wire.endTransmission();
    */

    wiringPiI2CWriteRegN(_fd, __reg, __dta, __len);
    // Try this version
    //wiringPiI2CWrite(_fd, __reg);
    //write(_fd, __dta, __len);

    
}

void I2C_CAN::IIC_CAN_SetReg(unsigned char __reg, unsigned char __dta)
{

    /*
    Wire.beginTransmission(IIC_ADDR);
    Wire.write(__reg);
    Wire.write(__dta);
    Wire.endTransmission();
    */
    wiringPiI2CWriteReg8(_fd, __reg, __dta);

}

bool I2C_CAN::IIC_CAN_GetReg(unsigned char __reg, unsigned char *__dta)
{
    /*
    Wire.beginTransmission(IIC_ADDR);
    Wire.write(__reg);
    Wire.endTransmission();
    Wire.requestFrom(IIC_ADDR, 1);
    
    while(Wire.available())
    {
        *__dta = Wire.read();
        return 1;
    }
    
    return 0;
    */

    // Write so it point to the register 
    wiringPiI2CWrite(_fd, __reg);
    I2C_sleep(MIN_WRITE_DELAY);


    // No read a byte back 
    int tmp = wiringPiI2CRead(_fd) ;

    /*
    // old veersion

    // returns data.byte & 0xFF (so only LSB) OR -1 if fails 
    int tmp = wiringPiI2CReadReg8(_fd, __reg);

    
  
    */
    if (tmp == -1) {
        return false;

    } else {
        *__dta = (uint8_t) tmp;  
        return true;
    }
}

bool I2C_CAN::IIC_CAN_GetReg(unsigned char __reg, int len, unsigned char *__dta)
{
    /*
    Wire.beginTransmission(IIC_ADDR);
    Wire.write(__reg);
    Wire.endTransmission();
    Wire.requestFrom(IIC_ADDR, len);
    
    int __len = 0;
    
    while(Wire.available())
    {
        __dta[__len++] = Wire.read();
    }



    return (len == __len);
    */

    // https://stackoverflow.com/questions/55976683/read-a-block-of-data-from-a-specific-registerfifo-using-c-c-and-i2c-in-raspb
    

    // Added this line 
    wiringPiI2CWrite(_fd, __reg);
    I2C_sleep(MIN_WRITE_DELAY);

    /*
    for (int i = 0; i < len; i++) {
       __dta[i] = wiringPiI2CRead(_fd);
    }
    */ 

    //https://raspberrypi.stackexchange.com/questions/87142/reading-multiple-bytes-in-raspberry-pi-over-i2c-using-wiring-pi-library
    // This seems significantly faster than read reg N
    read(_fd, __dta, len);

    //return true;

    
    // Is the * before __dta redunfance here? 
    
    //int tmp= wiringPiI2CReadRegN(_fd, __reg, (uint8_t*) __dta, len);  
    /*
    int tmp= wiringPiI2CReadRegN(_fd, __reg,  __dta, len);    

    if (tmp == -1) {
        return false;
    } else {
        return true; 
    }
    */
    
}




byte I2C_CAN::init_Mask(byte num, byte ext, unsigned long ulData)       // init Masks
{
    unsigned char dta[5];
    
    dta[0] = ext;
    dta[1] = 0xff & (ulData >> 24);
    dta[2] = 0xff & (ulData >> 16);
    dta[3] = 0xff & (ulData >> 8);
    dta[4] = 0xff & (ulData >> 0);
    
    unsigned char mask = (num == 0) ? REG_MASK0 : REG_MASK1;
    
    IIC_CAN_SetReg(mask, 5, dta);
    //delay(50);
    I2C_sleep(50000);
}

byte I2C_CAN::init_Filt(byte num, byte ext, unsigned long ulData)       // init filters
{
    unsigned char dta[5];
    
    dta[0] = ext;
    dta[1] = 0xff & (ulData >> 24);
    dta[2] = 0xff & (ulData >> 16);
    dta[3] = 0xff & (ulData >> 8);
    dta[4] = 0xff & (ulData >> 0);
    
    unsigned char filt = (7+num)*0x10;
    
    IIC_CAN_SetReg(filt, 5, dta);
    //delay(50);
    I2C_sleep(50000);
}

byte I2C_CAN::sendMsgBuf(unsigned long id, byte ext, byte rtr, byte len, byte *buf)     // send buf
{
    unsigned char dta[16];
    
    dta[0] = 0xff & (id >> 24);
    dta[1] = 0xff & (id >> 16);
    dta[2] = 0xff & (id >> 8);
    dta[3] = 0xff & (id >> 0);
    
    dta[4] = ext;
    dta[5] = rtr;
    
    dta[6] = len;
    
    for(int i=0; i<len; i++)
    {
        dta[7+i] = buf[i];
    }
    
    dta[15] = makeCheckSum(dta, 15);
    
    IIC_CAN_SetReg(REG_SEND, 16, dta);
}

byte I2C_CAN::sendMsgBuf(unsigned long id, byte ext, byte len, byte *buf)               // send buf
{
    sendMsgBuf(id, ext, 0, len, buf);

}

byte I2C_CAN::readMsgBuf(byte *len, byte *buf)                          // read buf
{
    readMsgBufID(&m_ID, len, buf);
}

byte I2C_CAN::readMsgBufID(unsigned long *ID, byte *len, byte *buf)     // read buf with object ID
{
    //IIC_CAN_GetReg(unsigned char __reg, int len, unsigned char *__dta)
    unsigned long id = 0;
    
    unsigned char dta[16];

    // For debug on checking if frame there 


    //unsigned long tic = micros();
    int tmp_num_frames = framesAvail();


    if (tmp_num_frames > 1) { 
        printf("Frames avail %i\n", tmp_num_frames);        
        //printf("time %i\n", (int) tframe);

    } else if (tmp_num_frames == 0) {
        printf(" Frames avail %i\n", tmp_num_frames);
        //printf("time %i\n", (int) tframe);
        I2C_sleep(800);
        /*
        while (tmp_num_frames == 0) {
            I2C_sleep(50);
            tmp_num_frames = framesAvail();
            printf("Frames avail %i, time %i\n", tmp_num_frames, (int) tframe);
        }
        */
    }
    
    
    // ALWAYS READ 16 BYTES
    IIC_CAN_GetReg(REG_RECV, 16, dta);  // 16 byte read 
    
    unsigned char __checksum = makeCheckSum(dta, 15);
 
    if(__checksum == dta[15])           // checksum ok
    {
        id = dta[0];
        id <<= 8;
        id += dta[1];
        id <<= 8;
        id += dta[2];
        id <<= 8;
        id += dta[3];
        
        *ID = id;
        
        m_ID  = id;
        m_EXT = dta[4];
        m_RTR = dta[5];
        
        *len = dta[6];   // NOTE: length gets set from the data 
        
        //Serial.print("readMsgBufID, len = ");
        //Serial.println(*len);
        
        if(*len > 8) {
            printf("message length too long\n");
            return 0;
        }
        
        for(int i=0; i<*len; i++)
            buf[i] = dta[7+i];
        
        return 1;
    }
    else 
    {
        
        // Is it possible this error is happening when there is nothing in the buffer?


        if (!clear_flag) {

            printf("Received data length %i\n", (int) dta[6]);
            printf("byte 0 %i\n", (int) dta[0]);
            printf("byte 1 %i\n", (int) dta[1]);
            printf("byte 2 %i\n", (int) dta[2]);
            printf("byte 3 %i\n", (int) dta[3]);
            printf("byte 4 %i\n", (int) dta[4]);
            printf("byte 5 %i\n", (int) dta[5]);
            printf("byte 6 %i\n", (int) dta[6]);

            printf("Checksum error on recv, computed checksum %#04x, recieved checksum %#04x\n", __checksum, dta[15]);
            

            printf("======\n");
            int num_frames = framesAvail();
            printf("Frames avail %i\n", num_frames);

            // This 
            if (num_frames == 1) {
                // This means we tried to read before it was available 
                readMsgBufID(&m_ID, len, buf);
            }   else {
                printf("Clearing buffer\n");
                clear_buffer();
            }

        }

        return 0;
    }
}

int I2C_CAN::framesAvail(void) {
    unsigned char num = 0;
    IIC_CAN_GetReg(REG_DNUM, &num);
    return (int) num;
}

byte I2C_CAN::checkReceive(void)                                        // if something received
{
    unsigned char num = 0;
    
    if(IIC_CAN_GetReg(REG_DNUM, &num)) // if true, return non zero
    {
        if(num > 0)
        {
            return CAN_MSGAVAIL;
        } 
    }
    
    return 0;
}

byte I2C_CAN::checkError(void)                                          // if something error
{
    return 0;
}

unsigned long I2C_CAN::getCanId(void)                                   // get can id when receive
{
    return m_ID;
}

byte I2C_CAN::isRemoteRequest(void)                                     // get RR flag when receive
{
    return m_RTR;
}

byte I2C_CAN::isExtendedFrame(void)                                     // did we recieve 29bit frame?
{
    return m_EXT;
}

unsigned char I2C_CAN::makeCheckSum(unsigned char *dta, int len)
{
    unsigned long sum = 0;
    for(int i=0; i<len; i++)sum += dta[i];
    
    if(sum > 0xff)
    {
        sum = ~sum;
        sum += 1;
    }
    
    sum  = sum & 0xff;
    return sum;
}


/**
 * 
 *  Erez Added 
 * 
 */

// May have old frames stored in the buffer, clear all of them on startup 

void I2C_CAN::clear_buffer(void) {
    unsigned char len = 0;
    unsigned char buf[8];

    clear_flag = true;

    while (CAN_MSGAVAIL == checkReceive()) {
        readMsgBuf(&len, buf); 
    }

    clear_flag = false; 

}


// Custom Replacement For I2C_sleep to check some bugs 
void I2C_sleep(uint32_t microseconds) { 

    struct timespec ts;
    struct timespec rem_time;

    ts.tv_sec = 0;
    ts.tv_nsec = 1000*microseconds;
    nanosleep(&ts, &rem_time);
}

// END FILE