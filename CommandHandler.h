#pragma once

/**
 * Execute the commands which are send from the Arduino-Simulator
 */

#include <Arduino.h>
#include <SPI.h>
#include "ArduinoLogger.h"

typedef uint8_t pin_size_t;
typedef int PinMode;

class CommandHandler {
  public:

    void receiveCommand(HardwareService &service) {
      HWCalls cmd = service.receiveCmd();
      switch(cmd){
        // GPIO
        case GpioPinMode:
          gpioPinMode(service);
          break;
        case GpioDigitalWrite:
          gpioDigitalWrite(service);
          break;
        case GpioDigitalRead:
          gpioDigitalRead(service);
          break;
        case GpioAnalogRead:
          gpioAnalogRead(service);
          break;
        case GpioAnalogReference:
          gpioAnalogReference(service);
          break;
        case GpioAnalogWrite:
          gpioAnalogWrite(service);
          break;
        case GpioTone:
          gpioTone(service);
          break;
        case GpioNoTone:
          gpioNoTone(service);
          break;
        case GpioPulseIn:
          gpioPulseIn(service);
          break;
        case GpioPulseInLong:
          gpioPulseInLong(service);
          break;
        // SPI
        case SpiTransfer:
          spiTransfer(service);
          break;
        case SpiTransfer8:
          spiTransfer(service);
          break;
        case SpiTransfer16:
          spiTransfer16(service);
          break;
        case SpiUsingInterrupt:
          spiUsingInterrupt(service);
          break;
        case SpiBeginTransaction:
          spiBeginTransaction(service);
          break;
        case SpiEndTransaction:
          spiEndTransaction(service);
          break;
        case SpiAttachInterrupt:
          spiAttachInterrupt(service);
          break;
        case SpiDetachInterrupt:
          spiDetachInterrupt(service);
          break;
        case SpiBegin:
          spiBegin(service);
          break;
        case SpiEnd:
          spiEnd(service);
          break;
        // Serial
        case SerialBegin:
          serialBegin(service);
          break;
        case SerialEnd:
          serialEnd(service);
        case SerialWrite:
          serialWrite(service);
          break;
        case SerialRead:
          serialRead(service);
          break;
        case SerialAvailable:
          serialAvailable(service);
          break;
        case SerialPeek:
          serialPeek(service);
          break;
        case SerialFlush:
          serialFlush(service);
          break;

        default:
          char msg[80];
          sprintf(msg,"Command not implemented: %d",cmd);
          Logger.log(Error, msg);
          break;
      }
    }
    
    protected:
      void gpioPinMode(HardwareService &service){
        pin_size_t pinNumber = service.receivePin();
        PinMode pinModeValue = (PinMode) service.receive8();

        pinMode(pinNumber, pinModeValue);

        char msg[80];
        sprintf(msg,"pinMode(%hhu,%d)",pinNumber,pinModeValue);
        Logger.log(Info,msg);
      }
  
      void gpioDigitalWrite(HardwareService &service){
        pin_size_t pinNumber = service.receivePin();
        uint8_t status = service.receive8();
        digitalWrite(pinNumber, status);
        
        char msg[80];
        sprintf(msg,"digitalWrite(%u,%u)",pinNumber, status);
        Logger.log(Info,msg); 
      }
  
      void gpioDigitalRead(HardwareService &service){
        pin_size_t pinNumber = service.receivePin();
        uint8_t status = digitalRead(pinNumber);

        char msg[80];
        sprintf(msg,"digitalRead(%d) -> %d",pinNumber, status);
        Logger.log(Info,msg);

        service.send(status);
        service.flush();
   
      }
  
      void gpioAnalogRead(HardwareService &service){
        //int analogRead(pin_size_t pinNumber);
        pin_size_t pinNumber = service.receivePin();
        int32_t value = analogRead(pinNumber);
        service.send(value);

        char msg[80];
        sprintf(msg,"analogRead(%d)",pinNumber);
        Logger.log(Info,msg);
        
      }
      
      void gpioAnalogReference(HardwareService &service){
        // void analogReference(uint8_t mode);
        uint8_t mode = service.receive8();
        //analogReference(mode);
        Logger.log(Error,"gpioAnalogReference","not supported");
      }

      void gpioAnalogWrite(HardwareService &service){
      // void analogWrite(pin_size_t pinNumber, int value);
        pin_size_t pinNumber = service.receivePin();
        int value = service.receiveInt();
        //analogWrite(pinNumber, value);
        Logger.log(Error,"gpioAnalogWrite","not supported");
      }
      
      void gpioTone(HardwareService &service){
        // void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
        pin_size_t pinNumber = service.receivePin();
        unsigned int frequency = service.receive32();
        unsigned long duration = service.receive64();
        //tone(pinNumber, frequency, duration = 0)
        Logger.log(Error,"gpioTone","not supported");
      }
      
      void gpioNoTone(HardwareService &service){
        //void noTone(uint8_t _pin);
        pin_size_t pinNumber = service.receivePin();
        //noTone(pinNumber);
        Logger.log(Error,"gpioNoTone","not supported");
      }
      
      void gpioPulseIn(HardwareService &service){
        // unsigned long pulseIn(pin_size_t pin, uint8_t state, unsigned long timeout);
        pin_size_t pinNumber = service.receivePin();
        uint8_t state = service.receive8();
        unsigned long timeout = service.receive64();
        pulseIn(pinNumber, state, timeout);        
 
        char msg[80];
        sprintf(msg,"pulseIn(%d,%d,%ld)",pinNumber,state, timeout);
        Logger.log(Info,msg);
      }
      
      void gpioPulseInLong(HardwareService &service){
        //unsigned long pulseInLong(pin_size_t pin, uint8_t state, unsigned long timeout);
        pin_size_t pinNumber = service.receivePin();
        uint8_t state = service.receive8();
        unsigned long timeout = service.receive64();
        pulseIn(pinNumber, state, timeout);

        char msg[80];
        sprintf(msg,"pulseIn(%d,%d,%ld)",pinNumber,state, timeout);
        Logger.log(Info,msg);
      }

      void spiTransfer(HardwareService &service){
        uint32_t size = service.receive32();
        uint8_t buffer[size];
        uint8_t data = service.receive(buffer, size);
        SPI.transfer(buffer, size); 
      }

      void spiTransfer8(HardwareService &service){
        uint8_t data = service.receive8();
        SPI.transfer(data); 
      }
      
      void spiTransfer16(HardwareService &service){
        uint16_t data = service.receive16();
        SPI.transfer16(data); 
      }
      
      void spiUsingInterrupt(HardwareService &service){
        uint16_t data = service.receive64();
        //SPI.usingInterrupt(data);         
        Logger.log(Error,"usingInterrupt","not supported");
      }
      
      void spiNotUsingInterrupt(HardwareService &service){
        uint16_t data = service.receive64();
        //SPI.notUsingInterrupt(data);                 
        Logger.log(Error,"notUsingInterrupt","not supported");
      }
      
      void spiBeginTransaction(HardwareService &service){
        uint32_t clck = service.receive32();
        uint8_t bitOrder = service.receive8();
        uint8_t dataMode = service.receive8();
        SPISettings settings(clck, bitOrder, dataMode);
        SPI.beginTransaction(settings);
      }
      
      void spiEndTransaction(HardwareService &service){
        SPI.endTransaction();
      }
      
      void spiAttachInterrupt(HardwareService &service){
        Logger.log(Error,"attachInterrupt","not supported");
        //SPI.attachInterrupt();        
      }
      
      void spiDetachInterrupt(HardwareService &service){
        Logger.log(Error,"detachInterrupt","not supported");
        //SPI.detachInterrupt();        
      }
      void spiBegin(HardwareService &service){
        SPI.begin();
      }
      void spiEnd(HardwareService &service){
        SPI.end();
      }

      // Serial
      void serialBegin(HardwareService &service){
         uint8_t no = service.receive8();
         size_t baud = service.receive64();
         HardwareSerial serial = getSerial(no);
         serial.begin(baud);                 
      }
      void serialEnd(HardwareService &service){
         uint8_t no = service.receive8();
         HardwareSerial serial = getSerial(no);
         serial.end();         
      }
      void serialWrite(HardwareService &service){
         uint8_t no = service.receive8();
         size_t len = service.receive64();
         HardwareSerial serial = getSerial(no);
         HardwareService::copy(serial, service.stream(), len);
      }
      void serialRead(HardwareService &service){
         uint8_t no = service.receive8();
         size_t len = service.receive64();
         HardwareSerial serial = getSerial(no);
         HardwareService::copy(service.stream(), serial, len);
      }
      void serialAvailable(HardwareService &service){
         uint8_t no = service.receive8();
         HardwareSerial s = getSerial(no);
         service.send(s.available());
         service.flush();        
      }
      void serialPeek(HardwareService &service){
         uint8_t no = service.receive8();
         HardwareSerial s = getSerial(no);
         service.send(s.peek());
         service.flush();
      }
      void serialFlush(HardwareService &service){        
         uint8_t no = service.receive8();
         HardwareSerial s = getSerial(no);
         s.flush();
      }
      HardwareSerial getSerial(int no){
        switch(no){
          case 0:
            return Serial;
          case 1:
            return Serial1;
          case 2:
            return Serial2;
        }
      }


};
