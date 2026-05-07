#pragma once

/**
 * Execute the commands which are sent from the Arduino-Simulator.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "ArduinoLogger.h"
#include "HardwareService.h"

/** @brief Pin number type used by the command protocol. */
typedef uint8_t pin_size_t;

/** @brief Pin mode type used by the command protocol. */
typedef int PinMode;

/**
 * @brief Executes incoming `HWCalls` commands on the local Arduino hardware APIs.
 *
 * `CommandHandler` reads a command from `HardwareService`, optionally invokes a
 * user callback, and dispatches to the corresponding GPIO/SPI/Serial handler.
 */
class CommandHandler {
 public:
  /**
  * @brief Callback type invoked before built-in command dispatch.
  *
  * Return `true` to mark the command as fully handled and skip default
  * dispatch. Return `false` to continue with built-in handling.
  *
  * @param cmd The received hardware call command.
  * @param handler Reference to the active `CommandHandler` instance.
  * @param service Reference to the active `HardwareService` transport.
  */
  using CommandCallback = bool (*)(HWCalls cmd, CommandHandler& handler,
                                   HardwareService& service);

  /**
  * @brief Registers a callback invoked for each received command.
  * @param cb Callback function pointer, or `nullptr` to disable callbacks.
  */
  void setCommandCallback(CommandCallback cb) { callback_ = cb; }

  /**
  * @brief Returns the currently registered command callback.
  * @return The callback function pointer or `nullptr` if not set.
  */
  CommandCallback commandCallback() const { return callback_; }

  /** @brief Removes the currently registered command callback. */
  void clearCommandCallback() { callback_ = nullptr; }

  /**
  * @brief Receives and executes a single command from the service stream.
  *
  * Processing order:
  * 1. Read one `HWCalls` value from `service`.
  * 2. If a callback is registered, invoke it.
  * 3. If callback returns `true`, stop processing.
  * 4. Otherwise execute the built-in command implementation.
  *
  * @param service Communication/serialization service carrying command payload.
  */
  void receiveCommand(HardwareService& service) {
    HWCalls cmd = service.receiveCmd();
    if (callback_ != nullptr) {
      if (callback_(cmd, *this, service)) {
        return;
      }
    }
    switch (cmd) {
      // I2C
      case I2cBegin0:
        i2cBegin0(service);
        break;
      case I2cBegin1:
        i2cBegin1(service);
        break;
      case I2cEnd:
        i2cEnd(service);
        break;
      case I2cSetClock:
        i2cSetClock(service);
        break;
      case I2cBeginTransmission:
        i2cBeginTransmission(service);
        break;
      case I2cEndTransmission1:
        i2cEndTransmission1(service);
        break;
      case I2cEndTransmission:
        i2cEndTransmission(service);
        break;
      case I2cRequestFrom3:
        i2cRequestFrom3(service);
        break;
      case I2cRequestFrom2:
        i2cRequestFrom2(service);
        break;
      case I2cOnReceive:
        i2cOnReceive(service);
        break;
      case I2cOnRequest:
        i2cOnRequest(service);
        break;
      case I2cWrite:
        i2cWrite(service);
        break;
      case I2cAvailable:
        i2cAvailable(service);
        break;
      case I2cRead:
        i2cRead(service);
        break;
      case I2cPeek:
        i2cPeek(service);
        break;

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
        spiTransfer8(service);
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
        break;
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

      default: {
        char msg[80];
        sprintf(msg, "Command not implemented: %d", (int)cmd);
        Logger.log(Error, msg);
        break;
      }
    }
  }


 protected:
  /** @brief Optional callback hook invoked before default command dispatch. */
  CommandCallback callback_ = nullptr;


  void gpioPinMode(HardwareService& service) {
    pin_size_t pinNumber = service.receivePin();
    PinMode pinModeValue = (PinMode)service.receive8();

    pinMode(pinNumber, pinModeValue);

    char msg[80];
    sprintf(msg, "pinMode(%hhu,%d)", pinNumber, pinModeValue);
    Logger.log(Info, msg);
  }

  void gpioDigitalWrite(HardwareService& service) {
    pin_size_t pinNumber = service.receivePin();
    uint8_t status = service.receive8();
    digitalWrite(pinNumber, status);

    char msg[80];
    sprintf(msg, "digitalWrite(%u,%u)", pinNumber, status);
    Logger.log(Info, msg);
  }

  void gpioDigitalRead(HardwareService& service) {
    pin_size_t pinNumber = service.receivePin();
    uint8_t status = digitalRead(pinNumber);

    char msg[80];
    sprintf(msg, "digitalRead(%d) -> %d", pinNumber, status);
    Logger.log(Info, msg);

    service.send(status);
    service.flush();
  }

  void gpioAnalogRead(HardwareService& service) {
    // int analogRead(pin_size_t pinNumber);
    pin_size_t pinNumber = service.receivePin();
    int32_t value = analogRead(pinNumber);
    service.send((int32_t)value);

    char msg[80];
    sprintf(msg, "analogRead(%d)", pinNumber);
    Logger.log(Info, msg);
  }

  void gpioAnalogReference(HardwareService& service) {
    (void)service.receive8();
    Logger.log(Error, "gpioAnalogReference", "not supported");
  }

  void gpioAnalogWrite(HardwareService& service) {
    (void)service.receivePin();
    (void)service.receiveInt();
    Logger.log(Error, "gpioAnalogWrite", "not supported");
  }

  void gpioTone(HardwareService& service) {
    (void)service.receivePin();
    (void)service.receive32();
    (void)service.receive64();
    Logger.log(Error, "gpioTone", "not supported");
  }

  void gpioNoTone(HardwareService& service) {
    (void)service.receivePin();
    Logger.log(Error, "gpioNoTone", "not supported");
  }

  void gpioPulseIn(HardwareService& service) {
    pin_size_t pinNumber = service.receivePin();
    uint8_t state = service.receive8();
    unsigned long timeout = (unsigned long)service.receive64();
    (void)pulseIn(pinNumber, state, timeout);

    char msg[80];
    sprintf(msg, "pulseIn(%d,%d,%lu)", pinNumber, state, timeout);
    Logger.log(Info, msg);
  }

  void gpioPulseInLong(HardwareService& service) {
    pin_size_t pinNumber = service.receivePin();
    uint8_t state = service.receive8();
    unsigned long timeout = (unsigned long)service.receive64();
    (void)pulseInLong(pinNumber, state, timeout);

    char msg[80];
    sprintf(msg, "pulseInLong(%d,%d,%lu)", pinNumber, state, timeout);
    Logger.log(Info, msg);
  }

  void spiTransfer(HardwareService& service) {
    uint32_t size = service.receive32();
    uint8_t buffer[64];

    while (size > 0) {
      size_t chunk = size > sizeof(buffer) ? sizeof(buffer) : size;
      size_t data = service.receive(buffer, (int)chunk);
      if (data == 0) {
        break;
      }
      SPI.transfer(buffer, data);
      size -= data;
    }
  }

  void spiTransfer8(HardwareService& service) {
    uint8_t data = service.receive8();
    SPI.transfer(data);
  }

  void spiTransfer16(HardwareService& service) {
    uint16_t data = service.receive16();
    SPI.transfer16(data);
  }

  void spiUsingInterrupt(HardwareService& service) {
    (void)service.receive64();
    Logger.log(Error, "usingInterrupt", "not supported");
  }

  void spiNotUsingInterrupt(HardwareService& service) {
    (void)service.receive64();
    Logger.log(Error, "notUsingInterrupt", "not supported");
  }

  void spiBeginTransaction(HardwareService& service) {
    uint32_t clck = service.receive32();
    uint8_t bitOrder = service.receive8();
    uint8_t dataMode = service.receive8();
    SPISettings settings(clck, bitOrder, dataMode);
    SPI.beginTransaction(settings);
  }

  void spiEndTransaction(HardwareService&) { SPI.endTransaction(); }

  void spiAttachInterrupt(HardwareService&) {
    Logger.log(Error, "attachInterrupt", "not supported");
  }

  void spiDetachInterrupt(HardwareService&) {
    Logger.log(Error, "detachInterrupt", "not supported");
  }

  void spiBegin(HardwareService&) { SPI.begin(); }

  void spiEnd(HardwareService&) { SPI.end(); }

  // Serial
  void serialBegin(HardwareService& service) {
    uint8_t no = service.receive8();
    size_t baud = (size_t)service.receive64();
    switch (no) {
      case 0:
        Serial.begin(baud);
        break;
#if defined(HAVE_HWSERIAL1)
      case 1:
        Serial1.begin(baud);
        break;
#endif
#if defined(HAVE_HWSERIAL2)
      case 2:
        Serial2.begin(baud);
        break;
#endif
      default:
        break;
    }
  }

  void serialEnd(HardwareService& service) {
    uint8_t no = service.receive8();
    switch (no) {
      case 0:
        Serial.end();
        break;
#if defined(HAVE_HWSERIAL1)
      case 1:
        Serial1.end();
        break;
#endif
#if defined(HAVE_HWSERIAL2)
      case 2:
        Serial2.end();
        break;
#endif
      default:
        break;
    }
  }

  void serialWrite(HardwareService& service) {
    uint8_t no = service.receive8();
    size_t len = (size_t)service.receive64();
    Stream* serial = getSerialStream(no);
    if (serial != nullptr) {
      HardwareService::copy(*serial, service.stream(), len);
    } else {
      // consume bytes even when target serial is unavailable
      uint8_t buffer[32];
      while (len > 0) {
        size_t chunk = len > sizeof(buffer) ? sizeof(buffer) : len;
        size_t readLen = service.receive(buffer, (int)chunk);
        if (readLen == 0) break;
        len -= readLen;
      }
    }
  }

  void serialRead(HardwareService& service) {
    uint8_t no = service.receive8();
    size_t len = (size_t)service.receive64();
    Stream* serial = getSerialStream(no);
    if (serial != nullptr) {
      HardwareService::copy(service.stream(), *serial, len);
    }
  }

  void serialAvailable(HardwareService& service) {
    uint8_t no = service.receive8();
    Stream* s = getSerialStream(no);
    service.send((int32_t)(s ? s->available() : 0));
    service.flush();
  }

  void serialPeek(HardwareService& service) {
    uint8_t no = service.receive8();
    Stream* s = getSerialStream(no);
    service.send((int32_t)(s ? s->peek() : -1));
    service.flush();
  }

  void serialFlush(HardwareService& service) {
    uint8_t no = service.receive8();
    Stream* s = getSerialStream(no);
    if (s != nullptr) {
      s->flush();
    }
  }

  Stream* getSerialStream(int no) {
    switch (no) {
      case 0:
        return &Serial;
#if defined(HAVE_HWSERIAL1)
      case 1:
        return &Serial1;
#endif
#if defined(HAVE_HWSERIAL2)
      case 2:
        return &Serial2;
#endif
      default:
        return nullptr;
    }
  }

  // I2C
  void i2cBegin0(HardwareService&) { Wire.begin(); }

  void i2cBegin1(HardwareService& service) {
    uint8_t address = service.receive8();
    Wire.begin((uint8_t)address);
  }

  void i2cEnd(HardwareService&) { Wire.end(); }

  void i2cSetClock(HardwareService& service) {
    uint32_t frequency = service.receive32();
    Wire.setClock(frequency);
  }

  void i2cBeginTransmission(HardwareService& service) {
    uint8_t address = service.receive8();
    Wire.beginTransmission(address);
  }

  void i2cEndTransmission1(HardwareService& service) {
    bool sendStop = service.receive8() != 0;
    uint8_t result = Wire.endTransmission(sendStop);
    service.send(result);
    service.flush();
  }

  void i2cEndTransmission(HardwareService& service) {
    uint8_t result = Wire.endTransmission();
    service.send(result);
    service.flush();
  }

  void i2cRequestFrom3(HardwareService& service) {
    uint8_t address = service.receive8();
    uint8_t quantity = service.receive8();
    bool sendStop = service.receive8() != 0;
    uint8_t received = Wire.requestFrom((int)address, (int)quantity, (int)sendStop);
    service.send(received);
    service.flush();
  }

  void i2cRequestFrom2(HardwareService& service) {
    uint8_t address = service.receive8();
    uint8_t quantity = service.receive8();
    uint8_t received = Wire.requestFrom((int)address, (int)quantity);
    service.send(received);
    service.flush();
  }

  void i2cOnReceive(HardwareService&) {
    Logger.log(Error, "i2cOnReceive", "remote callback not supported");
  }

  void i2cOnRequest(HardwareService&) {
    Logger.log(Error, "i2cOnRequest", "remote callback not supported");
  }

  void i2cWrite(HardwareService& service) {
    uint8_t data = service.receive8();
    uint8_t written = (uint8_t)Wire.write(data);
    service.send(written);
    service.flush();
  }

  void i2cAvailable(HardwareService& service) {
    int32_t result = (int32_t)Wire.available();
    service.send(result);
    service.flush();
  }

  void i2cRead(HardwareService& service) {
    int32_t result = (int32_t)Wire.read();
    service.send(result);
    service.flush();
  }

  void i2cPeek(HardwareService& service) {
    int32_t result = (int32_t)Wire.peek();
    service.send(result);
    service.flush();
  }  
};
