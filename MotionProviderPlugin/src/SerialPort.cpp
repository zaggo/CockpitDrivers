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
#include <poll.h>
#endif

// How long readBlocking() may block waiting for data before returning empty.
// Runs on a dedicated I/O thread (see ConnectionManager), so blocking here
// costs no render-frame time - this just bounds worst-case TX latency between
// read attempts.
static constexpr int kReadTimeoutMs = 20;

SerialPort::~SerialPort() {
    closePort();
}

#ifdef _WIN32
// ============ Windows Implementation ============

bool SerialPort::openPort(const std::string& devicePath, int baudRate) {
    // Close any existing handle first (locks internally). Configure a LOCAL
    // handle, then publish it under the lock once fully set up.
    closePort();

    // Windows serial port path format: \\\\.\\COMx
    std::string winPath = "\\\\.\\" + devicePath;

    // Open COM port
    HANDLE h = CreateFileA(
        winPath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,      // No sharing
        NULL,   // No security
        OPEN_EXISTING,
        0,      // Not overlapped I/O
        NULL
    );

    if (h == INVALID_HANDLE_VALUE) {
        return false;
    }

    // Configure COM port
    DCB dcbSerialParams;
    memset(&dcbSerialParams, 0, sizeof(dcbSerialParams));
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(h, &dcbSerialParams)) {
        CloseHandle(h);
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcbSerialParams)) {
        CloseHandle(h);
        return false;
    }

    // Read blocks for up to kReadTimeoutMs waiting for data (returns earlier
    // if data arrives sooner); this specific MAXDWORD/MAXDWORD/constant
    // combination is the documented way to get a bounded blocking read on
    // Windows instead of either an unbounded block or a busy-poll.
    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = kReadTimeoutMs;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(h, &timeouts)) {
        CloseHandle(h);
        return false;
    }

    // Flush buffers
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    { std::lock_guard<std::mutex> lk(portMutex_); hComm_ = h; }
    return true;
}

void SerialPort::closePort() {
    HANDLE h;
    { std::lock_guard<std::mutex> lk(portMutex_); h = hComm_; hComm_ = INVALID_HANDLE_VALUE; }
    if (h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
    }
}

bool SerialPort::isOpen() const {
    std::lock_guard<std::mutex> lk(portMutex_);
    return hComm_ != INVALID_HANDLE_VALUE;
}

bool SerialPort::writeBestEffort(const void* data, size_t len) {
    if (!data || len == 0) {
        return false;
    }
    HANDLE h;
    { std::lock_guard<std::mutex> lk(portMutex_); h = hComm_; }
    if (h == INVALID_HANDLE_VALUE) {
        return false;
    }

    DWORD bytesWritten = 0;
    if (!WriteFile(h, data, static_cast<DWORD>(len), &bytesWritten, NULL)) {
        closePort();
        return false;
    }

    return bytesWritten > 0;
}

size_t SerialPort::readBlocking(void* outBuf, size_t maxLen) {
    if (!outBuf || maxLen == 0) {
        return 0;
    }
    // Snapshot the handle under the lock, then release it before the (bounded)
    // blocking read so a concurrent writeBestEffort is never stalled.
    HANDLE h;
    { std::lock_guard<std::mutex> lk(portMutex_); h = hComm_; }
    if (h == INVALID_HANDLE_VALUE) {
        return 0;
    }

    DWORD bytesRead = 0;
    if (!ReadFile(h, outBuf, static_cast<DWORD>(maxLen), &bytesRead, NULL)) {
        // Read error
        closePort();
        return 0;
    }

    return static_cast<size_t>(bytesRead);
}

#else
// ============ POSIX (macOS/Linux) Implementation ============

bool SerialPort::openPort(const std::string& devicePath, int baudRate) {
    // Close any existing fd first (locks internally). Configure a LOCAL fd, then
    // publish it under the lock once fully set up.
    closePort();

    // Open device (non-blocking)
    int fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        return false;
    }

    // Configure terminal
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        close(fd);
        return false;
    }

    // Set baud rate
    int speed = baudToSpeed(baudRate);
    if (speed == -1) {
        close(fd);
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

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        close(fd);
        return false;
    }

    // Flush buffers
    tcflush(fd, TCIOFLUSH);

    { std::lock_guard<std::mutex> lk(portMutex_); fd_ = fd; }
    return true;
}

void SerialPort::closePort() {
    int fd;
    { std::lock_guard<std::mutex> lk(portMutex_); fd = fd_; fd_ = -1; }
    if (fd != -1) {
        tcflush(fd, TCIOFLUSH);
        close(fd);
    }
}

bool SerialPort::isOpen() const {
    std::lock_guard<std::mutex> lk(portMutex_);
    return fd_ != -1;
}

bool SerialPort::writeBestEffort(const void* data, size_t len) {
    if (!data || len == 0) {
        return false;
    }
    int fd;
    { std::lock_guard<std::mutex> lk(portMutex_); fd = fd_; }
    if (fd == -1) {
        return false;
    }

    ssize_t written = write(fd, data, len);

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

size_t SerialPort::readBlocking(void* outBuf, size_t maxLen) {
    if (!outBuf || maxLen == 0) {
        return 0;
    }
    // Snapshot the fd under the lock, then release it before the (bounded)
    // blocking poll/read so a concurrent writeBestEffort is never stalled.
    int fd;
    { std::lock_guard<std::mutex> lk(portMutex_); fd = fd_; }
    if (fd == -1) {
        return 0;
    }

    // fd is opened O_NONBLOCK, so use poll() to actually wait (low CPU)
    // for up to kReadTimeoutMs instead of busy-polling read().
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int pollResult = poll(&pfd, 1, kReadTimeoutMs);
    if (pollResult <= 0) {
        return 0;  // Timeout or error - nothing to read right now
    }

    ssize_t bytesRead = read(fd, outBuf, maxLen);

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