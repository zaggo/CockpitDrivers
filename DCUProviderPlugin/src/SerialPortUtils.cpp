#include "SerialPortUtils.h"

#ifdef _WIN32
#include <windows.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#else
#include <glob.h>
#endif

#ifdef _WIN32
// Numeric COM-number sort so COM12 lands after COM2, not after COM1.
static int comNumber(const std::string& name) {
    if (name.size() > 3 && (name.compare(0, 3, "COM") == 0 || name.compare(0, 3, "com") == 0))
        return std::atoi(name.c_str() + 3);
    return 0;
}
#endif

std::vector<std::string> enumerateSerialPorts() {
    std::vector<std::string> ports;
    
#ifdef _WIN32
    // Windows: read the live port map from the registry. Every present serial
    // device registers itself under SERIALCOMM, whether or not anyone has it open.
    //
    // The previous approach probed COM1..COM256 with CreateFile. That silently
    // dropped every port that was already open - including the one this plugin
    // itself is connected to, so the status window lost the active port as soon
    // as it was re-enumerated while connected. Opening ports as a probe also
    // toggles DTR on the ones it does open, which resets any Arduino sitting on
    // a neighbouring COM port.
    HKEY hKey = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        for (DWORD index = 0;; ++index) {
            char valueName[256];
            DWORD valueNameLen = sizeof(valueName);
            BYTE data[256];
            DWORD dataLen = sizeof(data);
            DWORD type = 0;
            LONG rc = RegEnumValueA(hKey, index, valueName, &valueNameLen, nullptr,
                                    &type, data, &dataLen);
            if (rc == ERROR_NO_MORE_ITEMS)
                break;
            if (rc != ERROR_SUCCESS || type != REG_SZ)
                continue;
            // REG_SZ data is not guaranteed to be NUL-terminated.
            std::string portName(reinterpret_cast<const char*>(data),
                                 strnlen(reinterpret_cast<const char*>(data), dataLen));
            if (!portName.empty())
                ports.push_back(portName);
        }
        RegCloseKey(hKey);
    }
    std::sort(ports.begin(), ports.end(), [](const std::string& a, const std::string& b) {
        int na = comNumber(a), nb = comNumber(b);
        return na != nb ? na < nb : a < b;
    });
#else
    // macOS/Linux: use glob to find /dev/cu.* or /dev/ttyUSB*
    glob_t glob_result;
    // Look for /dev/cu.* devices (macOS)
    if (glob("/dev/cu.*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            ports.emplace_back(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);
    }
#endif
    
    return ports;
}
