/*
        HardwareService.h
        Copyright (c) 2025 Phil Schatzmann. All right reserved.

        This library is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.

        This library is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.

        You should have received a copy of the GNU Lesser General Public
        License along with this library; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#pragma once

#include "EmulatorLogger.h"
#include "Stream.h"

/**
 * @brief Command identifiers exchanged between emulator and target.
 *
 * Values are serialized on the transport stream and decoded by
 * `CommandHandler`.
 */

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
  GpioAnalogWriteFrequency,
  GpioAnalogWriteResolution,
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
 * @class HardwareService
 * @brief Stream-based serializer/deserializer for hardware protocol messages.
 *
 * This type tunnels GPIO/SPI/Serial/I2C-like request and response payloads over
 * a generic Arduino `Stream`. It provides typed `send()` and `receive*()`
 * helpers plus endian conversion utilities.
 */

class HardwareService {
 public:
  /** @brief Constructs an unbound service (no stream configured). */
  HardwareService() {}

  /** @brief Constructs a service bound to the provided stream. */
  explicit HardwareService(Stream& str) { setStream(&str); }

  /** @brief Constructs a service bound to the provided stream pointer. */
  explicit HardwareService(Stream* str) { setStream(str); }

  /**
   * @brief Sets the underlying transport stream.
   * @param str Stream used for all reads/writes.
   */
  void setStream(Stream* str) { io = str; }

  /**
   * @brief Sends a command identifier.
   * @param call Protocol command value.
   */
  void send(HWCalls call) {
    uint16_t val = (uint16_t)call;
    io->write((uint8_t*)&val, sizeof(uint16_t));
  }

  /** @brief Sends an unsigned 8-bit value. */
  void send(uint8_t data) { io->write((uint8_t*)&data, sizeof(data)); }

  /** @brief Sends a signed int value. */
  void send(int data) { send((int32_t)data); }

  /** @brief Sends an unsigned int value. */
  void send(unsigned int data) { send((uint32_t)data); }

  /** @brief Sends an unsigned 16-bit value (little-endian on wire). */
  void send(uint16_t dataIn) {
    uint16_t data = swap_uint16(dataIn);
    io->write((uint8_t*)&data, sizeof(data));
  }

  /** @brief Sends an unsigned 32-bit value (little-endian on wire). */
  void send(uint32_t dataIn) {
    uint32_t data = swap_uint32(dataIn);
    io->write((uint8_t*)&data, sizeof(data));
  }

  /** @brief Sends an unsigned 64-bit value (little-endian on wire). */
  void send(uint64_t dataIn) {
    uint64_t data = swap_uint64(dataIn);
    io->write((uint8_t*)&data, sizeof(data));
  }

  /** @brief Sends a signed 32-bit value (little-endian on wire). */
  void send(int32_t dataIn) {
    int32_t data = swap_int32(dataIn);
    io->write((uint8_t*)&data, sizeof(data));
  }
  /** @brief Sends a signed 64-bit value (little-endian on wire). */
  void send(int64_t dataIn) {
    int64_t data = swap_int64(dataIn);
    io->write((uint8_t*)&data, sizeof(data));
  }

  /** @brief Sends a boolean value. */
  void send(bool data) { io->write((uint8_t*)&data, sizeof(data)); }

  /**
   * @brief Sends a raw byte buffer.
   * @param data Pointer to source bytes.
   * @param len Number of bytes.
   */
  void send(void* data, size_t len) { io->write((uint8_t*)data, len); }

  /** @brief Flushes the underlying stream. */
  void flush() { io->flush(); }

  /**
   * @brief Receives and decodes one command identifier.
   * @return Decoded `HWCalls` value.
   */
  HWCalls receiveCmd() {
    uint16_t result;
    size_t len = io->readBytes((char*)&result, sizeof(uint16_t));
    if (len != sizeof(uint16_t) && len != 0) {
      EmulatorLogger.log(Error, "receiveCmd Could not read all data");
    }

    return (HWCalls)swap_uint16(result);
  }

  /** @brief Receives a pin number (`uint8_t`). */
  uint8_t receivePin() { return receive8(); }

  /** @brief Receives an unsigned 16-bit value. */
  uint16_t receive16() {
    uint16_t result;
    blockingRead((char*)&result, sizeof(uint16_t));
    return swap_uint16(result);
  }

  /** @brief Receives an unsigned 32-bit value. */
  uint32_t receive32() {
    uint32_t result;
    blockingRead((char*)&result, sizeof(uint32_t));
    return swap_uint32(result);
  }

  /** @brief Receives an unsigned 64-bit value. */
  uint64_t receive64() {
    uint64_t result;
    blockingRead((char*)&result, sizeof(uint64_t));
    return swap_uint64(result);
  }

  /** @brief Receives an unsigned 8-bit value. */
  uint8_t receive8() {
    uint8_t result;
    blockingRead((char*)&result, sizeof(uint8_t));
    return result;
  }

  /** @brief Receives a signed 32-bit value. */
  int32_t receiveInt() { return (int32_t)receive32(); }

  /**
   * @brief Receives raw bytes into caller-provided buffer.
   * @param data Target buffer.
   * @param len Requested byte count.
   * @return Number of bytes read.
   */
  uint16_t receive(void* data, int len) {
    return blockingRead((char*)data, len);
  }

  /** @brief Returns the underlying stream. */
  Stream& stream() { return *io; }

  /** @brief Returns the underlying stream (const). */
  const Stream& stream() const { return *io; }

  /**
   * @brief Copies a fixed number of bytes from one stream to another.
   * @return Number of bytes copied.
   */
  static size_t copy(Stream& out, Stream& in, size_t len) {
    uint8_t buffer[64];
    size_t copied = 0;
    while (copied < len) {
      size_t chunk = (len - copied) > sizeof(buffer) ? sizeof(buffer) : (len - copied);
      size_t readLen = in.readBytes((char*)buffer, chunk);
      if (readLen == 0) {
        break;
      }
      out.write(buffer, readLen);
      copied += readLen;
    }
    return copied;
  }

  /**
   * @brief Checks whether a transport stream is configured.
   * @return `true` when `setStream()` has been called with non-null stream.
   */
  operator bool() { return io != nullptr; }

 protected:
  /** @brief Underlying transport stream. */
  Stream* io = nullptr;
  /** @brief Cached host endianness flag. */
  bool isLittleEndian = !is_big_endian();
  /** @brief Default blocking read timeout in milliseconds. */
  int timeout_ms = 1000;

  /**
   * @brief Performs a blocking read until all bytes are received or timeout.
   * @param data Destination buffer.
   * @param len Number of bytes requested.
   * @param timeout Timeout in milliseconds.
   * @return Number of bytes actually read.
   */
  uint16_t blockingRead(void* data, int len, int timeout = 1000) {
    int offset = 0;
    long start = millis();
    while (offset < len && (millis() - start) < timeout) {
      int n = io->readBytes((char*)data + offset, len - offset);
      offset += n;
    }
    return offset;
  }

  /** @brief Checks if host byte order is big-endian. */
  bool is_big_endian(void) {
    union {
      uint32_t i;
      char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1;
  }

  /** @brief Byte-swaps an unsigned 16-bit value when needed. */
  uint16_t swap_uint16(uint16_t val) {
    if (isLittleEndian) return val;
    return (val << 8) | (val >> 8);
  }

  /** @brief Byte-swaps a signed 16-bit value when needed. */
  int16_t swap_int16(int16_t val) {
    if (isLittleEndian) return val;
    return (val << 8) | ((val >> 8) & 0xFF);
  }

  /** @brief Byte-swaps an unsigned 32-bit value when needed. */
  uint32_t swap_uint32(uint32_t val) {
    if (isLittleEndian) return val;
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
  }

  /** @brief Byte-swaps a signed 32-bit value when needed. */
  int32_t swap_int32(int32_t val) {
    if (isLittleEndian) return val;
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | ((val >> 16) & 0xFFFF);
  }

  /** @brief Byte-swaps a signed 64-bit value when needed. */
  int64_t swap_int64(int64_t val) {
    if (isLittleEndian) return val;
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) |
          ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) |
          ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | ((val >> 32) & 0xFFFFFFFFULL);
  }

  /** @brief Byte-swaps an unsigned 64-bit value when needed. */
  uint64_t swap_uint64(uint64_t val) {
    if (isLittleEndian) return val;
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) |
          ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) |
          ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | (val >> 32);
  }
};

