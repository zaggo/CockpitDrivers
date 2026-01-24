#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    
    // Prevent copying
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    
    /// Opens a serial port with the specified device path and baud rate.
    /// Returns true on success, false on failure.
    bool openPort(const std::string& devicePath, int baudRate);
    
    /// Closes the serial port.
    void closePort();
    
    /// Returns true if the port is open.
    bool isOpen() const;
    
    /// Non-blocking write (best effort).
    /// If the write buffer is full (EAGAIN/EWOULDBLOCK), returns false.
    /// Other errors close the port and return false.
    /// Returns true if at least one byte was written.
    bool writeBestEffort(const void* data, size_t len);
    
    /// Non-blocking read.
    /// Returns the number of bytes read (0 if no data available).
    size_t readNonBlocking(void* outBuf, size_t maxLen);
    
private:
#ifdef _WIN32
    HANDLE hComm_ = INVALID_HANDLE_VALUE;
#else
    /// Converts a baud rate integer to termios speed constant.
    static int baudToSpeed(int baud);
    int fd_ = -1;
#endif
};