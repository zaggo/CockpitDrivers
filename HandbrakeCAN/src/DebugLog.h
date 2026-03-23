#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#include "Configuration.h"

#define DEBUGLOG_PORT Serial

#if DEBUGLOG_ENABLE
    #define DEBUGLOG_INIT(x)    DEBUGLOG_PORT.begin(x)
    #define DEBUGLOG_PRINT(x)   DEBUGLOG_PORT.print(x)
    #define DEBUGLOG_PRINTLN(x) DEBUGLOG_PORT.println(x)
#else
    #define DEBUGLOG_INIT(x)
    #define DEBUGLOG_PRINT(x)
    #define DEBUGLOG_PRINTLN(x)
#endif

#endif // DEBUGLOG_H