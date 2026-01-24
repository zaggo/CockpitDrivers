#include "SerialPort.h"

#include <cstring>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#endif

SerialPort::~SerialPort() {
    closePort();
}

#ifdef _WIN32
// ============ Windows Implementation ============

bool SerialPort::openPort(const std::string& devicePath, int baudRate) {
    // Already open?
    if (hComm_ != INVALID_HANDLE_VALUE) {
        closePort();
    }
    
    // Windows serial port path format: \\\\.\\COMx
    std::string winPath = "\\\\.\\" + devicePath;
    
    // Open COM port
    hComm_ = CreateFileA(
        winPath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,      // No sharing
        NULL,   // No security
        OPEN_EXISTING,
        0,      // Not overlapped I/O
        NULL
    );
    
    if (hComm_ == INVALID_HANDLE_VALUE) {
        return false;
    }
    
    // Configure COM port
    DCB dcbSerialParams;
    memset(&dcbSerialParams, 0, sizeof(dcbSerialParams));
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hComm_, &dcbSerialParams)) {
        closePort();
        return false;
    }
    
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    
    if (!SetCommState(hComm_, &dcbSerialParams)) {
        closePort();
        return false;
    }
    
    // Set timeouts for non-blocking operation
    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    
    if (!SetCommTimeouts(hComm_, &timeouts)) {
        closePort();
        return false;
    }
    
    // Flush buffers
    PurgeComm(hComm_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    
    return true;
}

void SerialPort::closePort() {
    if (hComm_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm_);
        hComm_ = INVALID_HANDLE_VALUE;
    }
}

bool SerialPort::isOpen() const {
    return hComm_ != INVALID_HANDLE_VALUE;
}

bool SerialPort::writeBestEffort(const void* data, size_t len) {
    if (!isOpen() || !data || len == 0) {
        return false;
    }
    
    DWORD bytesWritten = 0;
    if (!WriteFile(hComm_, data, static_cast<DWORD>(len), &bytesWritten, NULL)) {
        closePort();
        return false;
    }
    
    return bytesWritten > 0;
}

size_t SerialPort::readNonBlocking(void* outBuf, size_t maxLen) {
    if (!isOpen() || !outBuf || maxLen == 0) {
        return 0;
    }
    
    DWORD bytesRead = 0;
    if (!ReadFile(hComm_, outBuf, static_cast<DWORD>(maxLen), &bytesRead, NULL)) {
        // Read error
        closePort();
        return 0;
    }
    
    return static_cast<size_t>(bytesRead);
}

#else
// ============ POSIX (macOS/Linux) Implementation ============

bool SerialPort::openPort(const std::string& devicePath, int baudRate) {
    // Already open?
    if (fd_ != -1) {
        closePort();
    }
    
    // Open device (non-blocking)
    fd_ = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
        return false;
    }
    
    // Configure terminal
    struct termios options;
    if (tcgetattr(fd_, &options) != 0) {
        closePort();
        return false;
    }
    
    // Set baud rate
    int speed = baudToSpeed(baudRate);
    if (speed == -1) {
        closePort();
        return false;
    }
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    
    // Raw mode: no echo, no canonical processing
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    
    // Non-blocking read with timeout
    options.c_cc[VMIN] = 0;   // Return immediately, even if no data
    options.c_cc[VTIME] = 0;  // No timeout
    
    // Hardware flow control
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        closePort();
        return false;
    }
    
    // Flush buffers
    tcflush(fd_, TCIOFLUSH);
    
    return true;
}

void SerialPort::closePort() {
    if (fd_ != -1) {
        tcflush(fd_, TCIOFLUSH);
        close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const {
    return fd_ != -1;
}

bool SerialPort::writeBestEffort(const void* data, size_t len) {
    if (!isOpen() || !data || len == 0) {
        return false;
    }
    
    ssize_t written = write(fd_, data, len);
    
    // EAGAIN/EWOULDBLOCK = buffer full, try later (best effort)
    if (written == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // Would block, skip this write
        }
        // Other errors
        closePort();
        return false;
    }
    
    // Partial write is ok in best-effort mode
    return written > 0;
}

size_t SerialPort::readNonBlocking(void* outBuf, size_t maxLen) {
    if (!isOpen() || !outBuf || maxLen == 0) {
        return 0;
    }
    
    ssize_t bytesRead = read(fd_, outBuf, maxLen);
    
    if (bytesRead == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;  // No data available
        }
        // Other errors
        closePort();
        return 0;
    }
    
    return static_cast<size_t>(bytesRead);
}

int SerialPort::baudToSpeed(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        default:      return -1;
    }
}

#endif  // _WIN32