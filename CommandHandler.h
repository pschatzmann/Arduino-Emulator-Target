#pragma once

/**
 * Execute the commands which are send from the Arduino-Simulator
 */

#include <Arduino.h>
#include "ArduinoLogger.h"

typedef uint8_t pin_size_t;
typedef int PinMode;

class CommandHandler {
  public:

    void receiveCommand(HardwareService &service) {
      HWCalls cmd = service.receiveCmd();
      switch(cmd){
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
        Logger.log(Error,"gpioAnalogReference","not implemented");
      }

      void gpioAnalogWrite(HardwareService &service){
      // void analogWrite(pin_size_t pinNumber, int value);
        pin_size_t pinNumber = service.receivePin();
        int value = service.receiveInt();
        //analogWrite(pinNumber, value);
        Logger.log(Error,"gpioAnalogWrite","not implemented");
      }
      
      void gpioTone(HardwareService &service){
        // void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
        pin_size_t pinNumber = service.receivePin();
        unsigned int frequency = service.receive32();
        unsigned long duration = service.receive64();
        //tone(pinNumber, frequency, duration = 0)
        Logger.log(Error,"gpioTone","not implemented");
    }
      
      void gpioNoTone(HardwareService &service){
        //void noTone(uint8_t _pin);
        pin_size_t pinNumber = service.receivePin();
        //noTone(pinNumber);
        Logger.log(Error,"gpioNoTone","not implemented");
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

};
