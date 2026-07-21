#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <mutex>

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
    
    /// Blocks the calling thread for up to kReadTimeoutMs waiting for incoming
    /// data, then returns whatever is available (0 on timeout with no data).
    /// Intended to be called from a dedicated I/O thread, not from a
    /// render-frame-synced callback - use as a real (low-CPU) blocking wait,
    /// not a busy-poll.
    size_t readBlocking(void* outBuf, size_t maxLen);
    
private:
    // Guards the OS handle (fd_/hComm_) so the RX thread (readBlocking/isOpen)
    // and the TX thread (writeBestEffort/closePort) can't race on it. The
    // blocking poll/read runs on a handle SNAPSHOT taken under this lock and
    // released before the syscall, so a slow read never stalls a concurrent write.
    mutable std::mutex portMutex_;
#ifdef _WIN32
    HANDLE hComm_ = INVALID_HANDLE_VALUE;
#else
    /// Converts a baud rate integer to termios speed constant.
    static int baudToSpeed(int baud);
    int fd_ = -1;
#endif
};