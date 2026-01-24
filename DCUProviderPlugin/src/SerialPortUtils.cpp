#include "SerialPortUtils.h"
#include <glob.h>

std::vector<std::string> enumerateSerialPorts() {
    std::vector<std::string> ports;
    glob_t glob_result;
    // Look for /dev/cu.* devices
    if (glob("/dev/cu.*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            ports.emplace_back(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);
    }
    return ports;
}
