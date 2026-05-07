#pragma once

/**
 * A simple Logger which supports the output to a stream and log levels.
 */
#include <Arduino.h>
#include <Stream.h>
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief Supported log levels.
 *
 * Messages are emitted when `current_level >= configured_level`.
 */
enum LogLevel { Debug, Info, Warning, Error };

/**
 * @brief Lightweight logger writing to an Arduino `Stream`.
 *
 * The logger is disabled until `setLogger()` is called.
 */
class ArduinoLogger {
 public:
  /** @brief Constructs a logger in disabled state. */
  ArduinoLogger() : log_stream_ptr(nullptr), log_level(Error) {}
  /** @brief Virtual destructor. */
  ~ArduinoLogger() {}

  /**
   * @brief Activates logging output.
   * @param out Target stream (e.g. `Serial`).
   * @param level Minimum level to emit.
   */
  virtual void setLogger(Stream& out, LogLevel level = Error) {
    this->log_stream_ptr = &out;
    this->log_level = level;
  }

  /**
   * @brief Indicates if logging is active.
   * @return `true` when an output stream was configured.
   */
  virtual bool isLogging() { return log_stream_ptr != nullptr; }

  /**
   * @brief Logs a printf-style formatted message.
   * @param current_level Message severity.
   * @param format `printf`-style format string.
   * @param ... Optional format arguments.
   */
  virtual void log(LogLevel current_level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog(current_level, format, args);
    va_end(args);
  }

  /**
   * @brief Logs a printf-style formatted message from a `va_list`.
   * @param current_level Message severity.
   * @param format `printf`-style format string.
   * @param args Variable argument list.
   */
  virtual void vlog(LogLevel current_level, const char* format, va_list args) {
    if (log_stream_ptr == nullptr || current_level < log_level) {
      return;
    }

    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    log_stream_ptr->println(buffer);
    log_stream_ptr->flush();
  }

 protected:
  /** @brief Output stream used for logging, or `nullptr` if disabled. */
  Stream* log_stream_ptr;
  /** @brief Minimum enabled log level. */
  LogLevel log_level;
};

/** @brief Global logger instance used throughout the library. */
static ArduinoLogger Logger;
