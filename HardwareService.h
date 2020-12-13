#pragma once

#include "Stream.h"
#include "ArduinoLogger.h"
  
/**
 * We virtualize the hardware and send the requests and replys over
 * a stream.
 *
 **/

enum HWCalls {
    I2cBegin0,
    I2cBegin1,
    I2cEnd,
    I2cSetClock,
    I2cBeginTransmission,
    I2cEndTransmission1,
    I2cEndTransmission,
    I2cRequestFrom3,
    I2cRequestFrom2,
    I2cOnReceive,
    I2cOnRequest,
    I2cWrite,
    I2cAvailable,
    I2cRead,
    I2cPeek,
    SpiTransfer,
    SpiTransfer8,
    SpiTransfer16,
    SpiUsingInterrupt,
    SpiNotUsingInterrupt,
    SpiBeginTransaction,
    SpiEndTransaction,
    SpiAttachInterrupt,
    SpiDetachInterrupt,
    SpiBegin,
    SpiEnd,
    GpioPinMode,
    GpioDigitalWrite,
    GpioDigitalRead,
    GpioAnalogRead,
    GpioAnalogReference,
    GpioAnalogWrite,
    GpioTone,
    GpioNoTone,
    GpioPulseIn,
    GpioPulseInLong,
    SerialBegin,
    SerialEnd,
    SerialWrite,
    SerialRead,
    SerialAvailable,
    SerialPeek,
    SerialFlush,
    I2sSetup,
    I2sBegin3,
    I2sBegin2,
    I2sEnd,
    I2sAvailable,
    I2sRead,
    I2sPeek,
    I2sFlush,
    I2sWrite,
    I2sAvailableForWrite,
    I2sSetBufferSize
};

/**
 * Stream over which we tunnel the SPI, I2C, I2S and GPIO messages. Since most embedded divices
 * are little endian we communicate in little endian!
 *
 **/
    
class HardwareService {
    
  public:
    HardwareService(){
    }

    HardwareService(Stream &str){
      setStream(&str);
    }
   
    HardwareService(void *str){
        setStream((Stream*)str);
    }
    
    void setStream(Stream *str){
        io = str;
    }
    
    void send(HWCalls call){
        io->write((uint8_t*)&call,sizeof(call));
    }
    
    void send(uint8_t data){
        io->write((uint8_t*)&data,sizeof(data));
    }

    void send(uint16_t dataIn){
        uint16_t data = swap_uint16(dataIn);
        io->write((uint8_t*)&data,sizeof(data));
    }

    void send(uint32_t dataIn){
        uint32_t data = swap_uint32(dataIn);
        io->write((uint8_t*)&data,sizeof(data));
    }

    void send(int dataIn){
        int32_t data = swap_int32(dataIn);
        io->write((uint8_t*)&data,sizeof(data));
    }

    void send(bool data){
        io->write((uint8_t*)&data,sizeof(data));
    }
    
    // for size_t
    void send(uint64_t dataIn){
        uint64_t data = swap_uint64(dataIn);
        io->write((uint8_t*)&data,sizeof(data));
    }

    void send(void *data, size_t len){
        io->write((uint8_t*)data, len);
    }
    
    void flush() {
        io->flush();
    }

    HWCalls receiveCmd(){
        uint16_t result;
        size_t len =  io->readBytes((char*)&result, sizeof(uint16_t));
        if (len!=sizeof(uint16_t) && len!=0){
            Logger.log(Error,"receiveCmd","Could not read all data");
        }
        
        return (HWCalls)swap_uint16(result);
    }

    uint8_t receivePin(){
        return receive8();
    }

    uint8_t receive8(){
        uint8_t result;
        size_t len = io->readBytes((char*)&result, sizeof(uint8_t));
        if (len!=sizeof(uint8_t)){
            Logger.log(Error,"receive8","Could not read all data");
        }
        return result;
    }
    
    
    uint16_t receive16(){
        uint16_t result;
        size_t len = io->readBytes((char*)&result, sizeof(uint16_t));
        if (len!=sizeof(uint16_t)){
            Logger.log(Error,"receive16","Could not read all data");
        }
        return swap_uint16(result);
    }

    uint32_t receive32(){
        uint16_t result;
        size_t len = io->readBytes((char*)&result, sizeof(uint32_t));
        if (len!=sizeof(uint32_t)){
            Logger.log(Error,"receive32","Could not read all data");
        }
        return swap_uint32(result);
    }
    
    uint64_t receive64(){
        uint16_t result;
        size_t len = io->readBytes((char*)&result, sizeof(uint64_t));
        if (len!=sizeof(uint64_t)){
            Logger.log(Error,"receive64","Could not read all data");
        }
        return swap_uint64(result);
    }

    int receiveInt(){
        int64_t result;
        size_t len = io->readBytes((char*)&result, sizeof(int64_t));
        if (len!=sizeof(int64_t)){
            Logger.log(Error,"receiveInt","Could not read all data");
        }
        return swap_int64(result);
    }
    
    size_t receive(void* data, int len){
        return io->readBytes((char*)data,len);
    }

    // provides access to the stream which is used to communicate
    Stream& stream() {
      return *io;
    }

    // copies the data from a source stream to a target stream
    static size_t copy(Stream &to, Stream& from, size_t len){
      uint8_t buffer[512];  
      while(len>0){
         size_t recLen = len > 512 ? 512 : len;
         size_t readLen = from.readBytes(buffer, recLen);
         to.write(buffer,readLen);
         len -= readLen;
      }
      return len;
    }

    
   
  protected:
    Stream *io;
    bool isLittleEndian = !is_big_endian();
    
    // check if the system is big endian
    bool is_big_endian(void){
        union {
            uint32_t i;
            char c[4];
        } bint = {0x01020304};

        return bint.c[0] == 1; 
    }


    //! Byte swap unsigned short
    uint16_t swap_uint16( uint16_t val ) {
        if (isLittleEndian) return val;
        return (val << 8) | (val >> 8 );
    }

    //! Byte swap short
    int16_t swap_int16( int16_t val ) {
        if (isLittleEndian) return val;
        return (val << 8) | ((val >> 8) & 0xFF);
    }

    //! Byte swap unsigned int
    uint32_t swap_uint32( uint32_t val ){
        if (isLittleEndian) return val;
        val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
        return (val << 16) | (val >> 16);
    }

    //! Byte swap int
    int32_t swap_int32( int32_t val ){
        if (isLittleEndian) return val;
        val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF ); 
        return (val << 16) | ((val >> 16) & 0xFFFF);
    }

    int64_t swap_int64( int64_t val ){
        if (isLittleEndian) return val;
        val = ((val << 8) & 0xFF00FF00FF00FF00ULL ) | ((val >> 8) & 0x00FF00FF00FF00FFULL );
        val = ((val << 16) & 0xFFFF0000FFFF0000ULL ) | ((val >> 16) & 0x0000FFFF0000FFFFULL );
        return (val << 32) | ((val >> 32) & 0xFFFFFFFFULL);
    }

    uint64_t swap_uint64( uint64_t val ){
        if (isLittleEndian) return val;
        val = ((val << 8) & 0xFF00FF00FF00FF00ULL ) | ((val >> 8) & 0x00FF00FF00FF00FFULL );
        val = ((val << 16) & 0xFFFF0000FFFF0000ULL ) | ((val >> 16) & 0x0000FFFF0000FFFFULL );
        return (val << 32) | (val >> 32);
    }
 
};
