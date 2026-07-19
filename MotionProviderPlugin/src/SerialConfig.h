#pragma once

struct SerialConfig {
    int    baud   = 115200;
    double rateHz = 60.0;   // BFF frame stream rate
    static SerialConfig defaults() { return SerialConfig{}; }
};
