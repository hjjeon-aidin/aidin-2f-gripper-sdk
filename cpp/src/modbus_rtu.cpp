// Aidin Gripper SDK - self-contained Modbus RTU master (no external library).
#include "aidin/modbus_rtu.hpp"

#include <cerrno>
#include <cstring>
#include <thread>

#if defined(_WIN32)
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <poll.h>
  #include <termios.h>
  #include <unistd.h>
#endif

namespace aidin {

namespace {

// Modbus spec recommends a fixed 1.75 ms inter-frame gap above 19200 bps;
// enforcing it between back-to-back transactions keeps the slave's frame
// delimiter happy during command bursts (e.g. moveTo = 4 writes).
constexpr auto kInterframe = std::chrono::microseconds(1750);

std::string sysError(const char* what) {
#if defined(_WIN32)
    return std::string(what) + ": win32 error " + std::to_string(::GetLastError());
#else
    return std::string(what) + ": " + std::strerror(errno);
#endif
}

#if defined(_WIN32)
// Windows requires "\\.\COM10" style for ports >= COM10; the prefixed form
// is always safe, so apply it to every COMx name.
std::string normalizePort(const std::string& port) {
    if (port.size() >= 3 &&
        (port[0] == 'C' || port[0] == 'c') &&
        (port[1] == 'O' || port[1] == 'o') &&
        (port[2] == 'M' || port[2] == 'm') &&
        port.compare(0, 4, "\\\\.\\") != 0) {
        return std::string("\\\\.\\") + port;
    }
    return port;
}
#endif

} // namespace

ModbusRtu::~ModbusRtu() { close(); }

uint16_t ModbusRtu::crc16(const uint8_t* data, size_t len) noexcept {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

void ModbusRtu::guardInterframe() noexcept {
    const auto since = Clock::now() - lastIo_;
    if (since < kInterframe) {
        std::this_thread::sleep_for(kInterframe - since);
    }
}

// ---------------------------------------------------------------------------
// Platform backends: open / close / flushInput / writeAll / readExact
// ---------------------------------------------------------------------------
#if defined(_WIN32)

bool ModbusRtu::open(const std::string& port, int baudrate, char parity) {
    close();
    const std::string name = normalizePort(port);
    HANDLE h = ::CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                             nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        lastError_ = sysError(("open " + port).c_str());
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(h, &dcb)) {
        lastError_ = sysError("GetCommState");
        ::CloseHandle(h);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baudrate);
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    switch (parity) {
        case 'N': case 'n': dcb.Parity = NOPARITY;   dcb.fParity = FALSE; break;
        case 'E': case 'e': dcb.Parity = EVENPARITY; dcb.fParity = TRUE;  break;
        case 'O': case 'o': dcb.Parity = ODDPARITY;  dcb.fParity = TRUE;  break;
        default:
            lastError_ = "unsupported parity";
            ::CloseHandle(h);
            return false;
    }
    dcb.fBinary       = TRUE;
    dcb.fOutxCtsFlow  = FALSE;
    dcb.fOutxDsrFlow  = FALSE;
    dcb.fDtrControl   = DTR_CONTROL_DISABLE;
    dcb.fRtsControl   = RTS_CONTROL_DISABLE;
    dcb.fOutX = dcb.fInX = FALSE;
    if (!::SetCommState(h, &dcb)) {
        lastError_ = sysError("SetCommState");
        ::CloseHandle(h);
        return false;
    }
    ::PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = h;
    lastIo_ = Clock::now();
    return true;
}

void ModbusRtu::close() noexcept {
    if (handle_) {
        ::CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
}

bool ModbusRtu::isOpen() const noexcept { return handle_ != nullptr; }

void ModbusRtu::flushInput() noexcept {
    if (handle_) ::PurgeComm(static_cast<HANDLE>(handle_), PURGE_RXCLEAR);
}

bool ModbusRtu::writeAll(const uint8_t* data, size_t len) {
    DWORD written = 0;
    if (!::WriteFile(static_cast<HANDLE>(handle_), data,
                     static_cast<DWORD>(len), &written, nullptr) ||
        written != len) {
        lastError_ = sysError("write");
        return false;
    }
    ::FlushFileBuffers(static_cast<HANDLE>(handle_));
    return true;
}

bool ModbusRtu::readExact(uint8_t* buf, size_t len, Clock::time_point deadline) {
    size_t got = 0;
    while (got < len) {
        const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - Clock::now());
        if (remain.count() <= 0) {
            lastError_ = "response timeout";
            return false;
        }
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout        = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = MAXDWORD;
        to.ReadTotalTimeoutConstant   = static_cast<DWORD>(remain.count());
        ::SetCommTimeouts(static_cast<HANDLE>(handle_), &to);

        DWORD n = 0;
        if (!::ReadFile(static_cast<HANDLE>(handle_), buf + got,
                        static_cast<DWORD>(len - got), &n, nullptr)) {
            lastError_ = sysError("read");
            return false;
        }
        if (n == 0) {
            lastError_ = "response timeout";
            return false;
        }
        got += n;
    }
    return true;
}

#else // POSIX ---------------------------------------------------------------

namespace {
speed_t toSpeed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
        default:     return 0;
    }
}
} // namespace

bool ModbusRtu::open(const std::string& port, int baudrate, char parity) {
    close();
    const speed_t sp = toSpeed(baudrate);
    if (sp == 0) {
        lastError_ = "unsupported baud rate: " + std::to_string(baudrate);
        return false;
    }
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        lastError_ = sysError(("open " + port).c_str());
        return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        lastError_ = sysError("tcgetattr");
        close();
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(CSTOPB | CRTSCTS);              // 1 stop bit, HW flow off
    tio.c_cflag  = (tio.c_cflag & ~CSIZE) | CS8;
    switch (parity) {
        case 'N': case 'n':
            tio.c_cflag &= ~PARENB;
            break;
        case 'E': case 'e':
            tio.c_cflag |= PARENB;
            tio.c_cflag &= ~PARODD;
            tio.c_iflag |= INPCK;
            break;
        case 'O': case 'o':
            tio.c_cflag |= (PARENB | PARODD);
            tio.c_iflag |= INPCK;
            break;
        default:
            lastError_ = "unsupported parity";
            close();
            return false;
    }
    tio.c_cc[VMIN]  = 0;                             // poll()-driven non-blocking reads
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        lastError_ = sysError("tcsetattr");
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    lastIo_ = Clock::now();
    return true;
}

void ModbusRtu::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool ModbusRtu::isOpen() const noexcept { return fd_ >= 0; }

void ModbusRtu::flushInput() noexcept {
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

bool ModbusRtu::writeAll(const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        const ssize_t n = ::write(fd_, data + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            lastError_ = sysError("write");
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    tcdrain(fd_);
    return true;
}

bool ModbusRtu::readExact(uint8_t* buf, size_t len, Clock::time_point deadline) {
    size_t got = 0;
    while (got < len) {
        const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - Clock::now());
        if (remain.count() <= 0) {
            lastError_ = "response timeout";
            return false;
        }
        pollfd pfd{fd_, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, static_cast<int>(remain.count()));
        if (pr < 0) {
            if (errno == EINTR) continue;
            lastError_ = sysError("poll");
            return false;
        }
        if (pr == 0) {
            lastError_ = "response timeout";
            return false;
        }
        const ssize_t n = ::read(fd_, buf + got, len - got);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            lastError_ = sysError("read");
            return false;
        }
        got += static_cast<size_t>(n);
    }
    return true;
}

#endif // platform backends

// ---------------------------------------------------------------------------
// Protocol layer (platform independent)
// ---------------------------------------------------------------------------

bool ModbusRtu::transact(uint8_t* req, size_t reqLenWoCrc,
                         uint8_t* resp, size_t normalLen) {
    if (!isOpen()) {
        lastError_ = "port not open";
        return false;
    }
    const uint16_t crc = crc16(req, reqLenWoCrc);
    req[reqLenWoCrc]     = static_cast<uint8_t>(crc & 0xFF);
    req[reqLenWoCrc + 1] = static_cast<uint8_t>(crc >> 8);

    guardInterframe();
    // A late response from a previously timed-out transaction may still be
    // sitting in the RX buffer — always discard it before a fresh request.
    flushInput();

    if (!writeAll(req, reqLenWoCrc + 2)) {
        lastIo_ = Clock::now();
        return false;
    }

    const auto deadline = Clock::now() + timeout_;
    bool ok = false;
    do {
        // header: [slave][func]
        if (!readExact(resp, 2, deadline)) break;
        if (resp[0] != req[0]) {
            lastError_ = "slave id mismatch";
            break;
        }
        if (resp[1] == (req[1] | 0x80)) {
            // exception frame: [slave][func|0x80][exc][crcL][crcH]
            if (!readExact(resp + 2, 3, deadline)) break;
            const uint16_t rc = crc16(resp, 3);
            if (resp[3] != (rc & 0xFF) || resp[4] != (rc >> 8)) {
                lastError_ = "crc error (exception frame)";
                break;
            }
            lastError_ = "modbus exception code " + std::to_string(resp[2]);
            break;
        }
        if (resp[1] != req[1]) {
            lastError_ = "function code mismatch";
            break;
        }
        if (!readExact(resp + 2, normalLen - 2, deadline)) break;
        const uint16_t rc = crc16(resp, normalLen - 2);
        if (resp[normalLen - 2] != (rc & 0xFF) || resp[normalLen - 1] != (rc >> 8)) {
            lastError_ = "crc error";
            break;
        }
        ok = true;
    } while (false);

    lastIo_ = Clock::now();
    return ok;
}

bool ModbusRtu::readHolding(uint8_t slave, uint16_t addr, uint16_t count, uint16_t* out) {
    if (count == 0 || count > 125) {
        lastError_ = "invalid register count";
        return false;
    }
    uint8_t req[8] = {
        slave, 0x03,
        static_cast<uint8_t>(addr >> 8),  static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count & 0xFF)};
    uint8_t resp[5 + 250];
    const size_t normalLen = 5 + 2 * static_cast<size_t>(count);
    if (!transact(req, 6, resp, normalLen)) return false;
    if (resp[2] != 2 * count) {
        lastError_ = "byte count mismatch";
        return false;
    }
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = static_cast<uint16_t>((resp[3 + 2 * i] << 8) | resp[4 + 2 * i]);
    }
    return true;
}

bool ModbusRtu::writeSingle(uint8_t slave, uint16_t addr, uint16_t value) {
    uint8_t req[8] = {
        slave, 0x06,
        static_cast<uint8_t>(addr >> 8),  static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    uint8_t resp[8];
    if (!transact(req, 6, resp, 8)) return false;
    if (std::memcmp(resp + 2, req + 2, 4) != 0) {
        lastError_ = "write echo mismatch";
        return false;
    }
    return true;
}

bool ModbusRtu::writeMultiple(uint8_t slave, uint16_t addr,
                              const uint16_t* values, uint16_t count) {
    if (count == 0 || count > 123) {
        lastError_ = "invalid register count";
        return false;
    }
    uint8_t req[9 + 246];
    req[0] = slave;
    req[1] = 0x10;
    req[2] = static_cast<uint8_t>(addr >> 8);
    req[3] = static_cast<uint8_t>(addr & 0xFF);
    req[4] = static_cast<uint8_t>(count >> 8);
    req[5] = static_cast<uint8_t>(count & 0xFF);
    req[6] = static_cast<uint8_t>(2 * count);
    for (uint16_t i = 0; i < count; ++i) {
        req[7 + 2 * i] = static_cast<uint8_t>(values[i] >> 8);
        req[8 + 2 * i] = static_cast<uint8_t>(values[i] & 0xFF);
    }
    uint8_t resp[8];
    return transact(req, 7 + 2 * static_cast<size_t>(count), resp, 8);
}

} // namespace aidin
