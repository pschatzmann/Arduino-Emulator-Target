#pragma once

/**
 * A simple Logger which supports the output to a stream and log levels.
 */
#include <Arduino.h>
#include <Stream.h>

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
   * @brief Logs a message if the level passes the configured threshold.
   * @param current_level Message severity.
   * @param str Primary message segment.
   * @param str1 Optional second segment.
   * @param str2 Optional third segment.
   */
  virtual void log(LogLevel current_level, const char* str,
                   const char* str1 = nullptr, const char* str2 = nullptr) {
    if (log_stream_ptr != nullptr && current_level >= log_level) {
      log_stream_ptr->print(str);
      if (str1 != nullptr) {
        log_stream_ptr->print(" ");
        log_stream_ptr->print(str1);
      }
      if (str2 != nullptr) {
        log_stream_ptr->print(" ");
        log_stream_ptr->print(str2);
      }
      log_stream_ptr->println();
      log_stream_ptr->flush();
    }
  }

 protected:
  /** @brief Output stream used for logging, or `nullptr` if disabled. */
  Stream* log_stream_ptr;
  /** @brief Minimum enabled log level. */
  LogLevel log_level;
};

/** @brief Global logger instance used throughout the library. */
static ArduinoLogger Logger;
